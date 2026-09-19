#include "MotionMatchingSearch.h"

#include "../Database/MotionMatchingDatabase.h"
#include "../Runtime/MotionMatchingSampler.h"
#include "Engine/Core/Log.h"
#include "Engine/Core/Math/Math.h"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>

namespace
{
    // N1b rev 3: baked and query features are RAW. The only stored stat is a
    // positive per-dim deviation (baker floors it at `dev > 0.1 ? dev : 1.0`),
    // so the divide floor below is only a last-resort guard and never
    // participates for valid databases.
    constexpr float MinimumStandardDeviation = 0.000001f;
    constexpr float SelfTestMaxCost = 0.0001f;

    // Straightness prior (P2: Game-owned tuning via settings, values preserved
    // from the 0.03/4.0 calibration): penalty = |clipTurnAngle| * Weight /
    // (1 + yawRate * YawScale). Recorded in the trace; brute-force, indexed
    // and replay share this one definition.
    // Phase 2 UE-parity verdict (measured): grouped L/R normalization did
    // NOT retire this — with weight 0, turn one-shots flood steady strafe/
    // diagonal phases (suite 2/12 vs 5-6/12). The penalty is load-bearing
    // for pool purity beyond asymmetry compensation, so it stays (rescaled
    // to rev-3 cost units below: same ratio against typical dissimilarity).
    // (Attempt 1 of the penalty-0 retune failed cleanly; code path kept.)
    // Rev-3 scale (N1b shrank dissimilarity ~100x, measured mean 0.095):
    // 180deg -> +0.094 straight — same blocking ratio as the old +9.4.

    // Straightness prior, SINGLE definition shared by the hot loop, the
    // bit-exact verifier and the test gate below. Always >= 0 (fabs), so
    // block pruning stays valid with no extra headroom anywhere it is used.
    // P2: weight/scale ride the per-query settings (Game tuning, trace-
    // recorded); callers pass settings fields verbatim, never literals.
    float TurnPenalty(float clipAngle, float queryYawRate, float weight, float yawScale)
    {
        const float a = clipAngle >= 0.0f ? clipAngle : -clipAngle;
        const float w = weight >= 0.0f ? weight : 0.0f;
        const float s = yawScale >= 0.0f ? yawScale : 0.0f;
        return a * w / (1.0f + Math::Max(queryYawRate, 0.0f) * s);
    }

    // Foot-lock prior (Wave feet): a firmly pinned foot must not trade for
    // a swinging one mid-plant (measured: lock transfers coincide with the
    // biggest world-Y foot jumps at switches). D1/PLAN �5.2: the prior fires
    // only while the playback lock is FIRM (H1b LockState == Locked). The pin
    // weight still scales it, but a non-firm Planting/Releasing ramp must not
    // steer candidate ranking � the visual ramp stays in playback.
    // Folded into ContinuityCost like the straightness prior (>= 0, so block
    // pruning stays valid with no extra headroom).
    // N1b rev 3: baked contacts are RAW flags (1 = planted), so planted iff
    // the raw value is >= 0.5 (no z-score decode).
    // Rev-3 scale: same hold ratio as the old 15.0 against typical costs.
    // (foot-ik branch: softened, see below.)
    constexpr float FootLockPenalty = 0.05f;
    float FootLockCost(
        const MotionMatchingDatabase& database,
        const float* bakedPose,
        int32 poseFeatureSize,
        const MotionMatchingSearchSettings& settings)
    {
        const bool firmL = settings.FirmLockL;
        const bool firmR = settings.FirmLockR;
        if (!firmL && !firmR)
            return 0.0f;
        const float lockL = Math::Clamp(settings.FootLockL, 0.0f, 1.0f);
        const float lockR = Math::Clamp(settings.FootLockR, 0.0f, 1.0f);
        float lockCost = 0.0f;
        // N1: contact offsets via channel metadata, never size-2/-1.
    // foot-ik branch (soft): 0.05, down from 0.15. The old value rivaled a
    // whole winner cost (0.01-0.2), so a firm pin effectively VETOED any
    // phase-mismatched candidate � flips waited for phase-aligned winners
    // (+0.3-0.7s measured). 0.05 still breaks ties toward contact agreement
    // without outvoting a clearly better pose/trajectory match.
        // Bounds + finite-value guards before decode (IsReadyForQuery checks
        // counts; values are verified here so a corrupt flag can never NaN
        // the cost and break pruning).
        const int32 dimL = PoseSearchSchema::GetContactFeatureOffset(0);
        const int32 dimR = PoseSearchSchema::GetContactFeatureOffset(1);
        if (firmL && lockL > 0.0f && dimL >= 0 && dimL < poseFeatureSize &&
            std::isfinite(bakedPose[dimL]) && bakedPose[dimL] < 0.5f)
        {
            lockCost += FootLockPenalty * lockL;
        }
        if (firmR && lockR > 0.0f && dimR >= 0 && dimR < poseFeatureSize &&
            std::isfinite(bakedPose[dimR]) && bakedPose[dimR] < 0.5f)
        {
            lockCost += FootLockPenalty * lockR;
        }
        return lockCost;
    }

    // N1 weight contract: schema weights are stored, never baked. The search
    // applies each block weight exactly ONCE. Negative inputs clamp to 0;
    // non-finite settings are rejected by the FindBest gate before the hot
    // loop.
    void GetSearchBlockWeights(
        const MotionMatchingSearchSettings& settings,
        float& outPoseWeight,
        float& outTrajectoryWeight)
    {
        outPoseWeight = Math::Max(settings.PoseWeight, 0.0f);
        outTrajectoryWeight = Math::Max(settings.TrajectoryWeight, 0.0f);
    }

    // Weight-scale core shared by the live pipeline (below) and the index
    // tree space: sum-normalized sqrt block weight per dim / deviation.
    void BuildWeightScales(
        const float* poseDeviations,
        int32 poseFeatureSize,
        const float* trajectoryDeviations,
        int32 trajectoryFeatureSize,
        float poseWeight,
        float trajectoryWeight,
        float* poseScale,
        float* trajectoryScale)
    {
        const float weightSum = poseWeight * static_cast<float>(poseFeatureSize) +
            trajectoryWeight * static_cast<float>(trajectoryFeatureSize);
        const float poseRoot = weightSum > 0.0f ? std::sqrt(poseWeight / weightSum) : 0.0f;
        const float trajectoryRoot = weightSum > 0.0f ? std::sqrt(trajectoryWeight / weightSum) : 0.0f;
        for (int32 i = 0; i < poseFeatureSize; i++)
        {
            const float deviation = Math::Max(poseDeviations[i], MinimumStandardDeviation);
            poseScale[i] = poseRoot / deviation;
        }
        for (int32 i = 0; i < trajectoryFeatureSize; i++)
        {
            const float deviation = Math::Max(trajectoryDeviations[i], MinimumStandardDeviation);
            trajectoryScale[i] = trajectoryRoot / deviation;
        }
    }

    // N1b rev 3 weight pipeline (UE PoseSearchDerivedData analogue): per-dim
    // base weight = PoseWeight for pose dims, TrajectoryWeight for trajectory
    // dims; sum-normalize across ALL dims; sqrt each; divide by the stored
    // per-dim deviation. The raw query and raw baked values are then scored as
    // (scale * (query - baked))^2. Baked features are never pre-scaled, so a
    // weight retune never needs a rebake. Shared by the hot pass, top-N,
    // SampleCost, the exact self-test probes and the verifier.
    void BuildQueryScales(
        const MotionMatchingDatabase& database,
        const MotionMatchingSearchSettings& settings,
        float* poseScale,
        float* trajectoryScale)
    {
        const int32 poseFeatureSize = database.GetPoseFeatureSize();
        const int32 trajectoryFeatureSize = database.GetTrajectoryFeatureSize();
        const Array<float>& poseDeviations = database.GetPoseFeatureDeviations();
        const Array<float>& trajectoryDeviations = database.GetTrajectoryFeatureDeviations();
        float poseWeight = 0.0f;
        float trajectoryWeight = 0.0f;
        GetSearchBlockWeights(settings, poseWeight, trajectoryWeight);
        BuildWeightScales(
            poseDeviations.Get(), poseFeatureSize,
            trajectoryDeviations.Get(), trajectoryFeatureSize,
            poseWeight, trajectoryWeight,
            poseScale, trajectoryScale);
    }

    // Multi-tag membership: a clip passes when its tag SET overlaps the
    // allowed set. Empty allowed = all (unchanged contract).
    bool TagAllowed(const Array<String>& allowedTags, const Array<String>& clipTags)
    {
        if (allowedTags.IsEmpty())
            return true;
        for (const String& tag : clipTags)
        {
            if (allowedTags.Contains(tag))
                return true;
        }
        return false;
    }

    // Single-tag overload for the partition prefilter (partition t passes
    // when its tag name is in the allowed set).
    bool TagAllowed(const Array<String>& allowedTags, const String& tag)
    {
        return allowedTags.IsEmpty() || allowedTags.Contains(tag);
    }

    // Multi-tag exclusion: a candidate is dropped when its tag set shares
    // ANY tag with the exclude set. Empty exclude disables outright.
    bool TagExcluded(const Array<String>& excludeTags, const Array<String>& clipTags)
    {
        if (excludeTags.IsEmpty())
            return false;
        for (const String& tag : clipTags)
        {
            if (excludeTags.Contains(tag))
                return true;
        }
        return false;
    }

    // Tag-partitioned sample lists for the coarse prefilter. Built lazily on
    // first query per database (in-memory, no DB format change): one
    // sample-index list per baked tag. The per-tag KD-trees (phase 1-2) were
    // REMOVED after measurement � exact 74-D trees barely prune (1.9x) and a
    // trajectory-only tier regressed recall; the linear block prefilter won
    // (sub-1ms, 0/1500 divergence). Brute force stays the reference backend
    // permanently (fallback + verify harness).
    // Reference shape: UE PoseSearch retrieval + exact rerank
    // (PoseSearchDerivedData.cpp / PoseSearchIndex.cpp); orangeduck motion
    // matching (https://github.com/orangeduck/Motion-Matching).
    struct TagPartitionCache
    {
        const MotionMatchingDatabase* Db = nullptr;
        int32 SampleCount = -1;
        int32 ClipCount = -1;
        Array<String> TagNames;
        Array<Array<int32>> SamplesByTag;
        Array<int32> ClipsByTag;
    };

    TagPartitionCache& GetTagPartitionStore()
    {
        static TagPartitionCache cache;
        return cache;
    }

    // Phase 4 scratch buffers: IndexedSearchPass runs per-frame, so it must
    // not allocate per query (GC churn showed up as a fat qms tail). Single
    // game-thread caller (same discipline as the static partition cache);
    // buffers grow once to the high-water mark and are reused.
    struct CoarseEntry
    {
        float Dist = MAX_float;
        int32 Sample = -1;
    };

    struct IndexedScratch
    {
        Array<int32> Candidates;
        Array<CoarseEntry> Coarse;
    };

    IndexedScratch& GetIndexedScratch()
    {
        static IndexedScratch scratch;
        return scratch;
    }

    // Phase 2 (KDTREE_PLAN.md): parallel-verify counters. The indexed path
    // must reproduce brute-force winners before any flip; during verify the
    // BRUTE-FORCE result always wins (caller overwrites), this only counts.
    // Benchmark (phase 4): verify also accumulates per-path native scan time
    // (indexed vs unstrided-brute on the SAME queries, back-to-back) so the
    // speedup is measured without policy/diagnostics noise.
    struct IndexedVerifyStats
    {
        int32 Verifies = 0;
        int32 Diverges = 0;
        int32 Fallbacks = 0;
        int32 Logged = 0;
        int32 SampleCount = -1;
        double IndexedMs = 0.0;
        double BruteMs = 0.0;
    };

    IndexedVerifyStats& GetIndexedVerifyStats()
    {
        static IndexedVerifyStats stats;
        return stats;
    }

    void VerifyIndexedResult(
        const MotionMatchingDatabase& database,
        bool indexedOk,
        const MotionMatchingResult& indexed,
        const MotionMatchingResult& reference,
        double indexedMs,
        double bruteMs)
    {
        IndexedVerifyStats& stats = GetIndexedVerifyStats();
        if (stats.SampleCount != database.GetSampleCount())
        {
            stats.Verifies = 0;
            stats.Diverges = 0;
            stats.Fallbacks = 0;
            stats.Logged = 0;
            stats.SampleCount = database.GetSampleCount();
            stats.IndexedMs = 0.0;
            stats.BruteMs = 0.0;
            LOG(Info, "MotionMatching indexed verify: armed on {} clips / {} samples; brute force wins during verify.",
                database.GetClipCount(), database.GetSampleCount());
        }
        if (!indexedOk || !indexed.IsValid)
        {
            stats.Fallbacks++;
            if (stats.Logged < 16)
            {
                stats.Logged++;
                LOG(Warning, "MotionMatching indexed verify: FALLBACK (indexed produced no winner, brute force sample={}).",
                    reference.SampleIndex);
            }
            return;
        }
        stats.Verifies++;
        const bool sameWinner = indexed.IsValid == reference.IsValid &&
            indexed.SampleIndex == reference.SampleIndex &&
            (!indexed.IsValid || Math::Abs(indexed.Cost - reference.Cost) <= 0.0001f);
        if (!sameWinner)
        {
            stats.Diverges++;
            if (stats.Logged < 16)
            {
                stats.Logged++;
                LOG(Warning, "MotionMatching indexed verify DIVERGE #{}: coarse sample={} clip={} cost={:.6f} vs brute sample={} clip={} cost={:.6f}.",
                    stats.Verifies,
                    indexed.SampleIndex,
                    indexed.ClipIndex,
                    indexed.Cost,
                    reference.SampleIndex,
                    reference.ClipIndex,
                    reference.Cost);
            }
        }
        if (stats.Verifies % 500 == 0)
        {
            const double avgIndexed = stats.Verifies > 0 ? stats.IndexedMs / (double)stats.Verifies : 0.0;
            const double avgBrute = stats.Verifies > 0 ? stats.BruteMs / (double)stats.Verifies : 0.0;
            LOG(Info, "MotionMatching indexed verify: {}/{} diverge ({} fallbacks) over {} queries; native scan avg indexed={:.3f}ms brute={:.3f}ms.",
                stats.Diverges, stats.Verifies, stats.Fallbacks, stats.Verifies + stats.Fallbacks,
                avgIndexed, avgBrute);
        }
        stats.IndexedMs += indexedMs;
        stats.BruteMs += bruteMs;
    }

    const TagPartitionCache& EnsureTagPartition(const MotionMatchingDatabase& database)
    {
        TagPartitionCache& cache = GetTagPartitionStore();
        const int32 sampleCount = database.GetSampleCount();
        const int32 clipCount = database.GetClipCount();
        if (cache.Db == &database && cache.SampleCount == sampleCount && cache.ClipCount == clipCount)
            return cache;

        const auto buildStart = std::chrono::high_resolution_clock::now();
        cache.Db = &database;
        cache.SampleCount = sampleCount;
        cache.ClipCount = clipCount;
        cache.TagNames = database.GetAnimationTags();
        const int32 tagCount = cache.TagNames.Count();
        cache.SamplesByTag.Resize(tagCount);
        cache.ClipsByTag.Resize(tagCount);
        for (int32 t = 0; t < tagCount; t++)
        {
            cache.SamplesByTag[t].Clear();
            cache.ClipsByTag[t] = 0;
        }
        const Array<int32>& sampleClipIndices = database.GetSampleClipIndices();
        for (int32 s = 0; s < sampleCount && s < sampleClipIndices.Count(); s++)
        {
            const int32 clipIndex = sampleClipIndices[s];
            if (clipIndex < 0 || clipIndex >= clipCount)
                continue;
            // Multi-tag: a sample belongs to EVERY tag in its clip's set.
            const Array<String>& set = database.GetClipTags(clipIndex);
            for (int32 t = 0; t < tagCount; t++)
            {
                if (set.Contains(cache.TagNames[t]))
                    cache.SamplesByTag[t].Add(s);
            }
        }
        // Clip counts per tag: one clip counts into every tag it carries.
        for (int32 c = 0; c < clipCount; c++)
        {
            const Array<String>& set = database.GetClipTags(c);
            for (int32 t = 0; t < tagCount; t++)
            {
                if (set.Contains(cache.TagNames[t]))
                    cache.ClipsByTag[t]++;
            }
        }
        const auto buildEnd = std::chrono::high_resolution_clock::now();
        const double buildMs = std::chrono::duration<double, std::milli>(buildEnd - buildStart).count();
        int32 accounted = 0;
        for (int32 t = 0; t < tagCount; t++)
            accounted += cache.SamplesByTag[t].Count();
        LOG(Info, "MotionMatching tag partition: {} clips / {} samples / {} tags, built in {:.2f} ms ({} samples accounted).",
            clipCount, sampleCount, tagCount, buildMs, accounted);
        for (int32 t = 0; t < tagCount; t++)
        {
            LOG(Info, "MotionMatching tag partition: tag='{}' clips={} samples={}.",
                cache.TagNames[t], cache.ClipsByTag[t], cache.SamplesByTag[t].Count());
        }
        return cache;
    }

    float TimeDistance(const MotionMatchingDatabase& database, int32 clipIndex, float a, float b)
    {
        float distance = Math::Abs(a - b);
        if (clipIndex < 0 || !database.GetClipLoop(clipIndex))
            return distance;
        const Array<float>& lengths = database.GetClipLengths();
        if (clipIndex >= lengths.Count() || lengths[clipIndex] <= 0.0f)
            return distance;
        const float length = lengths[clipIndex];
        distance = Math::Mod(distance, length);
        return Math::Min(distance, length - distance);
    }

    // ---- S1 shared cost terms (PLAN �7.2, �16 Search, �18.7) ----
    // One definition per term. SearchPass (FindBest), FindTopCandidates,
    // FullSampleCost (SampleCost + the brute-force verifier) all build their
    // costs from these helpers, so no path can drift from another. Turn and
    // foot terms are the existing TurnPenalty / FootLockCost above.
    //
    // Pruning invariant kept: every prior is non-negative (turn + foot +
    // continuity switch penalty) so a pruned candidate can never earn cost
    // back; the only negative terms (continuing bias, loop preference) get
    // explicit headroom added to the prune bound in SearchPass.

    // Pose block. With a finite pruneLimit it stops once the partial block
    // alone already exceeds the bound (remaining terms are >= 0). The scale
    // array already folds in sqrt(sum-normalized weight)/deviation, so the raw
    // query/baked delta is squared directly.
    float PoseBlockCost(
        const float* queryPose,
        const float* bakedPose,
        const float* poseScale,
        int32 poseFeatureSize,
        float pruneLimit)
    {
        float poseCost = 0.0f;
        for (int32 i = 0; i < poseFeatureSize; i++)
        {
            const float delta = poseScale[i] * (queryPose[i] - bakedPose[i]);
            poseCost += delta * delta;
            if (poseCost > pruneLimit)
                return poseCost;
        }
        return poseCost;
    }

    // Trajectory block, pruned against spentPoseCost + the bound.
    float TrajectoryBlockCost(
        const float* queryTrajectory,
        const float* bakedTrajectory,
        const float* trajectoryScale,
        int32 trajectoryFeatureSize,
        float spentPoseCost,
        float pruneLimit)
    {
        float trajectoryCost = 0.0f;
        for (int32 i = 0; i < trajectoryFeatureSize; i++)
        {
            const float delta = trajectoryScale[i] * (queryTrajectory[i] - bakedTrajectory[i]);
            trajectoryCost += delta * delta;
            if (spentPoseCost + trajectoryCost > pruneLimit)
                return trajectoryCost;
        }
        return trajectoryCost;
    }

    // Continuity block folds the switch penalty and the continuing bias so
    // Pose + Trajectory + Continuity equals Cost (telemetry + self-test).
    float ContinuityTerm(
        bool continuityApplies,
        bool clipDiffers,
        bool isContinuation,
        float continuityWeight,
        float bias)
    {
        if (continuityApplies && clipDiffers)
            return continuityWeight;
        if (isContinuation && bias > 0.0f)
            return -bias;
        return 0.0f;
    }

    // Rest-loop preference bonus (<= 0; SearchPass adds loopPreference to the
    // prune bound so a pruned candidate cannot lose to an unseen loop bonus).
    float LoopPreferenceTerm(bool isLoop, float loopPreference)
    {
        return (isLoop && loopPreference > 0.0f) ? -loopPreference : 0.0f;
    }

    // Natural continuation (same clip, near the expected playback time). One
    // definition shared by the hot loop and the exact reference.
    bool IsContinuationSample(
        const MotionMatchingDatabase& database,
        const MotionMatchingSearchSettings& settings,
        bool useTimeTerms,
        int32 currentClipIndex,
        int32 clipIndex,
        float sampleTime,
        float biasWindow)
    {
        return useTimeTerms && clipIndex == currentClipIndex && biasWindow > 0.0f &&
            TimeDistance(database, clipIndex, settings.ExpectedTime, sampleTime) <= biasWindow;
    }

    // S1 �18.6: furthest future trajectory target from the schema. A non-loop
    // sample whose future stencil reaches past the clip end baked a clamped
    // "stopped" future for playback; search must not rank it (see
    // MotionMatchingSampler::IsNonLoopFutureOutside). Loop clips return 0.
    float GetTrajectoryHorizon(const MotionMatchingDatabase& database)
    {
        float horizon = 0.0f;
        const Array<float>& times = database.GetTrajectorySampleTimes();
        for (int32 i = 0; i < times.Count(); i++)
            horizon = Math::Max(horizon, times[i]);
        return horizon;
    }

    // All-terms cost of one baked sample for a raw query. Pose + Trajectory +
    // Continuity/bias + turn + foot-lock + loop, built from the shared term
    // helpers. This is the exact reference the self-test verifier compares
    // FindBest against, and the body of the C# fast-path SampleCost (called
    // there with continuity/loop/foot disabled).
    float FullSampleCost(
        const MotionMatchingDatabase& database,
        const MotionMatchingSearchSettings& settings,
        const float* queryPose,
        const float* queryTrajectory,
        int32 currentClipIndex,
        int32 sampleIndex,
        bool applyTurnPenalty)
    {
        const int32 poseFeatureSize = database.GetPoseFeatureSize();
        const int32 trajectoryFeatureSize = database.GetTrajectoryFeatureSize();
        const Array<float>& poseFeatures = database.GetPoseFeatures();
        const Array<float>& trajectoryFeatures = database.GetTrajectoryFeatures();
        const Array<int32>& sampleClipIndices = database.GetSampleClipIndices();
        const Array<float>& sampleTimes = database.GetSampleTimes();
        const int32 clipCount = database.GetClipCount();

        // One scale pass per reference call: weight pipeline is O(dims) and
        // this path is used by the exact verifier/SampleCost only.
        float stackPoseScale[PoseSearchSchema::GetPoseFeatureSize()];
        float stackTrajectoryScale[PoseSearchSchema::GetTrajectoryFeatureSize()];
        Array<float> heapPoseScale, heapTrajectoryScale;
        float* poseScale = stackPoseScale;
        float* trajectoryScale = stackTrajectoryScale;
        if (poseFeatureSize > PoseSearchSchema::GetPoseFeatureSize())
        {
            heapPoseScale.Resize(poseFeatureSize);
            poseScale = heapPoseScale.Get();
        }
        if (trajectoryFeatureSize > PoseSearchSchema::GetTrajectoryFeatureSize())
        {
            heapTrajectoryScale.Resize(trajectoryFeatureSize);
            trajectoryScale = heapTrajectoryScale.Get();
        }
        BuildQueryScales(database, settings, poseScale, trajectoryScale);

        const float continuityWeight = Math::Max(settings.ContinuityWeight, 0.0f);
        const bool continuityApplies = currentClipIndex >= 0 && currentClipIndex < clipCount;
        const bool useTimeTerms = continuityApplies && settings.ExpectedTime >= 0.0f;
        const float bias = Math::Max(settings.ContinuingBias, 0.0f);
        const float biasWindow = Math::Max(settings.ContinuingWindow, 0.0f);
        const float loopPreference = Math::Max(settings.LoopPreference, 0.0f);

        const int32 clipIndex = (sampleIndex >= 0 && sampleIndex < sampleClipIndices.Count())
            ? sampleClipIndices[sampleIndex]
            : -1;
        const float* bakedPose = &poseFeatures[sampleIndex * poseFeatureSize];
        const float* bakedTrajectory = &trajectoryFeatures[sampleIndex * trajectoryFeatureSize];
        const bool isContinuation = IsContinuationSample(
            database, settings, useTimeTerms, currentClipIndex, clipIndex,
            sampleTimes[sampleIndex], biasWindow);
        const float poseCost = PoseBlockCost(
            queryPose, bakedPose, poseScale, poseFeatureSize, MAX_float);
        const float trajectoryCost = TrajectoryBlockCost(
            queryTrajectory, bakedTrajectory, trajectoryScale, trajectoryFeatureSize,
            poseCost, MAX_float);
        float continuityCost = ContinuityTerm(
            continuityApplies, clipIndex != currentClipIndex, isContinuation,
            continuityWeight, bias);
        if (applyTurnPenalty && settings.TurnPenaltyWeight > 0.0f)
            continuityCost += TurnPenalty(database.GetClipTurnAngle(clipIndex), settings.QueryYawRate,
                settings.TurnPenaltyWeight, settings.TurnPenaltyYawScale);
        continuityCost += FootLockCost(database, bakedPose, poseFeatureSize, settings);
        const float loopCost = LoopPreferenceTerm(database.GetClipLoop(clipIndex), loopPreference);
        return poseCost + trajectoryCost + continuityCost + loopCost;
    }

    // Array wrapper for the self-test exact probes (they build raw arrays).
    float FullSampleCost(
        const MotionMatchingDatabase& database,
        const Array<float>& queryPose,
        const Array<float>& queryTrajectory,
        const MotionMatchingSearchSettings& settings,
        int32 currentClipIndex,
        int32 sampleIndex,
        bool applyTurnPenalty)
    {
        return FullSampleCost(
            database, settings, queryPose.Get(), queryTrajectory.Get(),
            currentClipIndex, sampleIndex, applyTurnPenalty);
    }

    // Exact per-class minima over the tag+filter scope: full scan, no stride,
    // no pruning, raw pose+trajectory blocks only (no continuity, bias,
    // preference or margin � this is the tune surface for those terms, so it
    // must not contain them). Respects the loop filter (a filtered-out class
    // reports untouched init values) and tags; ignores lock, exclusion and
    // stride, which are playback concerns, not tune data. Costs ~one extra
    // pruned scan per query; the hot path stays pruned.
    void ComputeClassBests(
        const MotionMatchingDatabase& database,
        const float* queryPose,
        const float* queryTrajectory,
        const float* poseScale,
        const float* trajectoryScale,
        const Array<String>& allowedTags,
        const Array<String>& excludeTags,
        MotionLoopFilter loopFilter,
        int32 oneShotBand,
        float& bestLoopCost,
        int32& bestLoopSample,
        int32& bestLoopClip,
        int32& loopTotalCount,
        float& bestOneShotCost,
        int32& bestOneShotSample,
        int32& bestOneShotClip,
        int32& oneShotTotalCount)
    {
        bestLoopCost = MAX_float;
        bestLoopSample = -1;
        bestLoopClip = -1;
        loopTotalCount = 0;
        bestOneShotCost = MAX_float;
        bestOneShotSample = -1;
        bestOneShotClip = -1;
        oneShotTotalCount = 0;

        const int32 poseFeatureSize = database.GetPoseFeatureSize();
        const int32 trajectoryFeatureSize = database.GetTrajectoryFeatureSize();
        const int32 sampleCount = database.GetSampleCount();
        const int32 clipCount = database.GetClipCount();
        const Array<int32>& sampleClipIndices = database.GetSampleClipIndices();
        const Array<float>& poseFeatures = database.GetPoseFeatures();
        const Array<float>& trajectoryFeatures = database.GetTrajectoryFeatures();

        int32 cachedClip = -1;
        bool cachedLoop = false;
        int32 cachedBand = 0;
        // Tags are per-clip; samples arrive clip-ordered, so resolve the
        // tag check once per clip instead of per sample. Empty include scope
        // AND empty exclude set disable the check outright.
        const bool checkTags = !allowedTags.IsEmpty() || !excludeTags.IsEmpty();
        bool cachedTagOk = true;
        for (int32 sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
        {
            const int32 clipIndex = sampleClipIndices[sampleIndex];
            if (clipIndex < 0 || clipIndex >= clipCount)
                continue;
            if (clipIndex != cachedClip)
            {
                cachedClip = clipIndex;
                cachedLoop = database.GetClipLoop(clipIndex);
                cachedBand = database.GetClipSpeedBand(clipIndex);
                const Array<String>& clipTags = database.GetClipTags(clipIndex);
                cachedTagOk = !checkTags ||
                    (TagAllowed(allowedTags, clipTags) && !TagExcluded(excludeTags, clipTags));
            }
            if (loopFilter == MotionLoopFilter::LoopOnly && !cachedLoop)
                continue;
            if (loopFilter == MotionLoopFilter::OneShotOnly && cachedLoop)
                continue;
            if (oneShotBand != 0 && !cachedLoop && cachedBand != 0 &&
                cachedBand != oneShotBand)
                continue;
            if (!cachedTagOk)
                continue;

            // Raw blocks through the shared term helpers (no prior, no prune):
            // this is the class tune surface and must match FullSampleCost's
            // pose/trajectory terms exactly.
            const float* bakedPose = &poseFeatures[sampleIndex * poseFeatureSize];
            const float* bakedTrajectory = &trajectoryFeatures[sampleIndex * trajectoryFeatureSize];
            const float poseCost = PoseBlockCost(
                queryPose, bakedPose, poseScale, poseFeatureSize, MAX_float);
            const float trajectoryCost = TrajectoryBlockCost(
                queryTrajectory, bakedTrajectory, trajectoryScale, trajectoryFeatureSize,
                poseCost, MAX_float);
            const float cost = poseCost + trajectoryCost;

            // Strictly-less keeps the deterministic lowest-index tie-break.
            if (cachedLoop)
            {
                loopTotalCount++;
                if (cost < bestLoopCost)
                {
                    bestLoopCost = cost;
                    bestLoopSample = sampleIndex;
                    bestLoopClip = clipIndex;
                }
            }
            else
            {
                oneShotTotalCount++;
                if (cost < bestOneShotCost)
                {
                    bestOneShotCost = cost;
                    bestOneShotSample = sampleIndex;
                    bestOneShotClip = clipIndex;
                }
            }
        }
    }

    // Lightweight per-query readiness check. Full ValidateData() walks every
    // sample/tag array (62k samples in the GASP DB) and WaitForLoaded() blocks
    // the game thread by default � both are wrong on the 10 Hz hot path.
    // The editor self-test and bake still run full ValidateData().
    bool IsReadyForQuery(const MotionMatchingDatabase& database, String& error)
    {
        if (database.WaitForLoaded(0))
        {
            error = TEXT("Motion matching search skipped: database is still loading.");
            return false;
        }
        if (!database.IsBaked())
        {
            error = TEXT("Motion matching search failed: database has no baked samples. Bake it first.");
            return false;
        }
        const int32 poseFeatureSize = database.GetPoseFeatureSize();
        const int32 trajectoryFeatureSize = database.GetTrajectoryFeatureSize();
        const int32 sampleCount = database.GetSampleCount();
        const int32 clipCount = database.GetClipCount();
        if (sampleCount <= 0 || clipCount <= 0 ||
            database.GetPoseFeatures().Count() != sampleCount * poseFeatureSize ||
            database.GetTrajectoryFeatures().Count() != sampleCount * trajectoryFeatureSize ||
            database.GetPoseFeatureDeviations().Count() != poseFeatureSize ||
            database.GetTrajectoryFeatureDeviations().Count() != trajectoryFeatureSize ||
            database.GetSampleClipIndices().Count() != sampleCount ||
            database.GetSampleTimes().Count() != sampleCount)
        {
            error = TEXT("Motion matching search failed: database dimensions are inconsistent (rebake required).");
            return false;
        }
        return true;
    }

    void SearchPass(
        const MotionMatchingDatabase& database,
        const float* queryPose,
        const float* queryTrajectory,
        const float* poseScale,
        const float* trajectoryScale,
        const MotionMatchingSearchSettings& settings,
        int32 currentClipIndex,
        bool lockSwitch,
        const Array<String>& allowedTags,
        int32 stride,
        MotionMatchingResult& result)
    {
        const int32 poseFeatureSize = database.GetPoseFeatureSize();
        const int32 trajectoryFeatureSize = database.GetTrajectoryFeatureSize();
        const int32 sampleCount = database.GetSampleCount();
        const int32 clipCount = database.GetClipCount();
        const Array<int32>& sampleClipIndices = database.GetSampleClipIndices();
        const Array<float>& sampleTimes = database.GetSampleTimes();
        const Array<float>& poseFeatures = database.GetPoseFeatures();
        const Array<float>& trajectoryFeatures = database.GetTrajectoryFeatures();

        const float continuityWeight = Math::Max(settings.ContinuityWeight, 0.0f);
        const bool continuityApplies = currentClipIndex >= 0 && currentClipIndex < clipCount;
        const bool useExclusion = continuityApplies && settings.CurrentTime >= 0.0f &&
            settings.SameClipExclusionWindow > 0.0f;
        const float exclusionWindow = Math::Max(settings.SameClipExclusionWindow, 0.0f);
        // Time-based terms need a current clip AND a valid expectation.
        // Without playback (ExpectedTime < 0) the search is pure cost.
        const bool useTimeTerms = continuityApplies && settings.ExpectedTime >= 0.0f;
        const float bias = Math::Max(settings.ContinuingBias, 0.0f);
        const float biasWindow = Math::Max(settings.ContinuingWindow, 0.0f);
        const float switchMargin = Math::Max(settings.SwitchMargin, 0.0f);
        const float loopPreference = Math::Max(settings.LoopPreference, 0.0f);
        // Wave 1 UE-parity bans (all exclusion-only: they can never lower a
        // cost, so block pruning needs no extra headroom).
        const float twinBan = Math::Max(settings.TwinBanWindow, 0.0f);
        const float tailExclusion = Math::Max(settings.LoopTailExclusion, 0.0f);
        const bool useReselectBan = settings.ReselectClip >= 0 && settings.ReselectClip < clipCount &&
            settings.ReselectBanWindow > 0.0f;
        const float reselectWindow = Math::Max(settings.ReselectBanWindow, 0.0f);
        const Array<float>& clipLengths = database.GetClipLengths();
        const bool haveLengths = clipLengths.Count() == clipCount;
        // S1 �7.2/�18.6 structural validity: non-loop candidates whose future
        // stencil reaches past the clip end baked a clamped "stopped" future,
        // so exclude them from ranking. Structural, never a tunable; loop
        // clips are cycle-aware and unaffected.
        const float trajectoryHorizon = GetTrajectoryHorizon(database);
        const bool excludeNonLoopTail = settings.ExcludeNonLoopTail;
        // Per-frame rotating stride: consecutive queries cover disjoint
        // subsets ((frame % stride) start) so full coverage completes over
        // `stride` frames instead of quantizing on a cadence. Deterministic:
        // the cursor rides settings.QueryFrame (policy frame counter, reset
        // per play session). Continuation samples are exempt below � the hold
        // must survive every frame regardless of the cursor. (Rebuild tick.)
        const int32 queryFrame = settings.QueryFrame >= 0 ? settings.QueryFrame : 0;
        const int32 strideOffset = (stride > 1) ? (queryFrame % stride) : 0;

        float bestCost = MAX_float;
        float secondBestCost = MAX_float;
        float bestPoseCost = MAX_float;
        float bestTrajectoryCost = MAX_float;
        float bestContinuityCost = 0.0f;
        float bestLoopCost = 0.0f;
        int32 bestSample = -1;
        // Best in-window same-clip continuation (final cost, bias included).
        float bestContinueCost = MAX_float;
        float bestContinuePoseCost = 0.0f;
        float bestContinueTrajectoryCost = 0.0f;
        float bestContinueContinuityCost = 0.0f;
        float bestContinueLoopCost = 0.0f;
        int32 bestContinueSample = -1;
        // Loop flag is per-clip; samples arrive clip-ordered, so resolve once
        // per clip instead of per sample (one native call per clip total).
        // Loop-mode prefilter runs before tags: cheapest structural cut.
        // Tags resolve on the same clip-change tick (one String copy +
        // compare per clip); empty scope disables the check outright.
        const bool checkTags = !allowedTags.IsEmpty() || !settings.ExcludeTags.IsEmpty();
        int32 cachedLoopClip = -1;
        bool cachedLoopFlag = false;
        bool cachedTagOk = true;
        float cachedTurnAngle = 0.0f;
        int32 cachedSpeedBand = 0;
        // Start of the non-searchable tail for the cached clip (-1 = none).
        float cachedTailStart = -1.0f;
        const bool wantLoop = settings.LoopFilter == MotionLoopFilter::LoopOnly;
        const bool wantOneShot = settings.LoopFilter == MotionLoopFilter::OneShotOnly;
        // Onset gait filter: restrict ONE-SHOTs to the intended band (loops
        // never band-cut). 0 = Any.
        const int32 oneShotBand = settings.OneShotBand;
        // Scored-member counts per class (completed both blocks). Pruned
        // members never land here. Class MINIMA come from ComputeClassBests
        // (unpruned full scan), never from this loop.
        int32 loopScoredCount = 0;
        int32 oneShotScoredCount = 0;

        for (int32 sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
        {
            const int32 clipIndex = sampleClipIndices[sampleIndex];
            if (clipIndex < 0 || clipIndex >= clipCount)
                continue;
            if (clipIndex != cachedLoopClip)
            {
                cachedLoopClip = clipIndex;
                cachedLoopFlag = database.GetClipLoop(clipIndex);
                cachedTurnAngle = database.GetClipTurnAngle(clipIndex);
                cachedSpeedBand = database.GetClipSpeedBand(clipIndex);
                const Array<String>& clipTagSet = database.GetClipTags(clipIndex);
                cachedTagOk = !checkTags || (TagAllowed(allowedTags, clipTagSet) && !TagExcluded(settings.ExcludeTags, clipTagSet));
                // Loop-tail cutoff for this clip (non-loop only, runtime from
                // lengths + sample times, no rebake). Clips shorter than the
                // exclusion keep full scope so short one-shots stay winnable.
                cachedTailStart = -1.0f;
                if (tailExclusion > 0.0f && !cachedLoopFlag && haveLengths &&
                    clipLengths[clipIndex] > tailExclusion)
                    cachedTailStart = clipLengths[clipIndex] - tailExclusion;
            }
            if ((wantLoop && !cachedLoopFlag) || (wantOneShot && cachedLoopFlag))
                continue;
            // Onset gait filter: one-shots outside the intended band never
            // rank (at the onset instant the features carry almost no gait
            // information, so walk/run starts would otherwise coin-flip).
            if (oneShotBand != 0 && !cachedLoopFlag && cachedSpeedBand != 0 &&
                cachedSpeedBand != oneShotBand)
                continue;
            if (lockSwitch && clipIndex != currentClipIndex)
                continue;
            if (!cachedTagOk)
                continue;
            // Loop-tail non-searchable (UE ExcludeFromDatabase (0,-0.3)):
            // end stances of one-shots can never re-win on exact pose match.
            if (cachedTailStart >= 0.0f && sampleTimes[sampleIndex] > cachedTailStart)
                continue;
            // S1 �7.2/�18.6: a non-loop future target outside [0, length] is
            // excluded (T1 helper), never ranked: the baked feature is a
            // clamped "stopped" future. Loop is unaffected (cycle-aware).
            if (excludeNonLoopTail && haveLengths &&
                MotionMatchingSampler::IsNonLoopFutureOutside(
                    sampleTimes[sampleIndex], trajectoryHorizon, clipLengths[clipIndex], cachedLoopFlag))
                continue;
            // Reselect-history ban (UE PoseReselectHistory): the recently
            // played (clip,sample) neighborhood cannot re-win. Kills A->B->A
            // chatter between near-twins; legitimate returns at another phase
            // (outside the window) still win fairly.
            if (useReselectBan && clipIndex == settings.ReselectClip &&
                TimeDistance(database, clipIndex, settings.ReselectSampleTime, sampleTimes[sampleIndex]) <= reselectWindow)
                continue;
            // Natural continuation: same clip, near the expected playback
            // time. Exempt from temporal exclusion (it IS the frame playback
            // is heading to, not a reseek) and rewarded with the bias.
            const bool isContinuation = IsContinuationSample(
                database, settings, useTimeTerms, currentClipIndex, clipIndex,
                sampleTimes[sampleIndex], biasWindow);
            // Rotating-stride cursor: skip non-continuation samples outside
            // this query's subset. Continuations are exempt so the hold never
            // flickers on subset rotation; everything else is covered over
            // `stride` consecutive queries.
            if (stride > 1 && !isContinuation &&
                (sampleIndex % stride) != strideOffset)
                continue;
            // Same-segment twin ban (UE PoseJumpThresholdTime): same-clip
            // non-continuations just outside the continuation core (near-twin
            // frames of where playback is heading) are skipped. The true
            // continuation above stays exempt, so steady play-through is
            // unaffected � only hops to your own twin frames die.
            if (useTimeTerms && twinBan > 0.0f && clipIndex == currentClipIndex && !isContinuation &&
                TimeDistance(database, clipIndex, settings.ExpectedTime, sampleTimes[sampleIndex]) <= biasWindow + twinBan)
                continue;
            // Temporal exclusion: same-clip samples too close in time to the
            // current playback position can never win. Without this, a query
            // every 0.1s reseeks to a nearby frame on the same clip (often the
            // same phase �1 sample) and playback micro-stutters forever instead
            // of playing through. Real discontinuities (large jumps, clip
            // changes, end recovery) are outside the window and unaffected.
            if (useExclusion && clipIndex == currentClipIndex && !isContinuation)
            {
                if (TimeDistance(database, clipIndex, settings.CurrentTime, sampleTimes[sampleIndex]) <= exclusionWindow)
                    continue;
            }

            // Query is pre-normalized once by the caller: baked features are
            // already z-scored, so this is a plain squared distance. Prune
            // against secondBestCost (not bestCost): pruning at bestCost
            // discards candidates that could still be the correct second-best,
            // which made SecondBestCost telemetry wrong whenever the true
            // runner-up shared the pose block with the winner.
            // Bias+margin+loop-aware pruning: a continuation's final cost can
            // dip below its block sum by up to bias, the switch margin lets it
            // win even when it is not top-2 on blocks alone, and loop clips
            // gain up to loopPreference � so those prune limits are raised by
            // exactly the applicable bonuses (see margin post-mortem: without
            // headroom an early exact winner drops secondBest near zero and
            // the arbitration never sees the candidates). Others keep tight.
            const float pruneLimit = secondBestCost +
                (isContinuation ? (bias > 0.0f ? bias : 0.0f) + switchMargin : 0.0f) +
                (cachedLoopFlag && loopPreference > 0.0f ? loopPreference : 0.0f);
            const float* bakedPose = &poseFeatures[sampleIndex * poseFeatureSize];
            const float* bakedTrajectory = &trajectoryFeatures[sampleIndex * trajectoryFeatureSize];
            // Shared terms (S1): the pose block alone can already disqualify
            // against the runner-up threshold because every remaining term is
            // non-negative except the bias/loop bonuses, whose headroom is in
            // pruneLimit above.
            const float poseCost = PoseBlockCost(
                queryPose, bakedPose, poseScale, poseFeatureSize, pruneLimit);
            if (poseCost > pruneLimit)
                continue;
            const float trajectoryCost = TrajectoryBlockCost(
                queryTrajectory, bakedTrajectory, trajectoryScale, trajectoryFeatureSize,
                poseCost, pruneLimit);
            if (poseCost + trajectoryCost > pruneLimit)
                continue;

            // Continuity block folds penalty AND bias so Pose + Trajectory +
            // Continuity still equals Cost (self-test invariant + telemetry).
            // Straightness + foot-lock priors fold in too (both >= 0, so
            // pruning stays valid with no extra headroom).
            float continuityCost = ContinuityTerm(
                continuityApplies, clipIndex != currentClipIndex, isContinuation,
                continuityWeight, bias);
            if (settings.TurnPenaltyWeight > 0.0f)
                continuityCost += TurnPenalty(cachedTurnAngle, settings.QueryYawRate,
                    settings.TurnPenaltyWeight, settings.TurnPenaltyYawScale);
            continuityCost += FootLockCost(database, bakedPose, poseFeatureSize, settings);
            // Rest-loop bonus lives in its own telemetry block (a loop clip at
            // rest outranks one-shot end stances structurally, not via continuity).
            const float loopCost = LoopPreferenceTerm(cachedLoopFlag, loopPreference);
            const float cost = poseCost + trajectoryCost + continuityCost + loopCost;

            // Class COUNTS only: how many members of each class survived
            // pruning to full scoring. The best-per-class VALUES come from
            // ComputeClassBests (unpruned full scan below) — a pruned member's
            // true cost is unknowable here, so this loop must not report minima.
            if (cachedLoopFlag)
                loopScoredCount++;
            else
                oneShotScoredCount++;

            // Strictly-less comparison: the first (lowest-index) sample wins
            // ties, which keeps replay deterministic down to the sample.
            if (cost < bestCost)
            {
                secondBestCost = bestCost;
                bestCost = cost;
                bestSample = sampleIndex;
                bestPoseCost = poseCost;
                bestTrajectoryCost = trajectoryCost;
                bestContinuityCost = continuityCost;
                bestLoopCost = loopCost;
            }
            else if (cost < secondBestCost)
            {
                secondBestCost = cost;
            }
            if (isContinuation && cost < bestContinueCost)
            {
                bestContinueCost = cost;
                bestContinueSample = sampleIndex;
                bestContinuePoseCost = poseCost;
                bestContinueTrajectoryCost = trajectoryCost;
                bestContinueContinuityCost = continuityCost;
                bestContinueLoopCost = loopCost;
            }
        }

        // Switch margin: an other-clip winner replaces the best continuation
        // only by a clear margin. bestCost <= secondBestCost always holds, so
        // the demoted winner becomes the new second-best exactly.
        if (bestSample >= 0 && bestContinueSample >= 0 &&
            sampleClipIndices[bestSample] != currentClipIndex &&
            switchMargin > 0.0f && bestCost + switchMargin >= bestContinueCost)
        {
            secondBestCost = bestCost;
            bestCost = bestContinueCost;
            bestSample = bestContinueSample;
            bestPoseCost = bestContinuePoseCost;
            bestTrajectoryCost = bestContinueTrajectoryCost;
            bestContinuityCost = bestContinueContinuityCost;
            bestLoopCost = bestContinueLoopCost;
        }

        if (bestSample < 0)
            return;

        result.IsValid = true;
        result.SampleIndex = bestSample;
        result.ClipIndex = sampleClipIndices[bestSample];
        result.SampleTime = sampleTimes[bestSample];
        result.Cost = bestCost;
        result.SecondBestCost = secondBestCost;
        result.PoseCost = bestPoseCost;
        result.TrajectoryCost = bestTrajectoryCost;
        result.ContinuityCost = bestContinuityCost;
        result.LoopCost = bestLoopCost;
        // Class minima come from the unpruned full scan (exact over the
        // tag+filter scope), never from the pruned loop above.
        if (settings.CollectClassDiagnostics)
        {
            ComputeClassBests(
                database,
                queryPose,
                queryTrajectory,
                poseScale,
                trajectoryScale,
                allowedTags,
                settings.ExcludeTags,
                settings.LoopFilter,
                settings.OneShotBand,
                result.BestLoopCost,
                result.BestLoopSample,
                result.BestLoopClip,
                result.LoopTotalCount,
                result.BestOneShotCost,
                result.BestOneShotSample,
                result.BestOneShotClip,
                result.OneShotTotalCount);
        }
        result.LoopScoredCount = loopScoredCount;
        result.OneShotScoredCount = oneShotScoredCount;
    }

    // Coarse-prefilter query: linear block retrieval over allowed tags +
    // EXACT rerank with the same terms/filters as SearchPass (shared helpers
    // above: Pose/TrajectoryBlockCost, ContinuityTerm, TurnPenalty,
    // FootLockCost, LoopPreferenceTerm, IsContinuationSample). Anything this
    // path cannot decide falls back to SearchPass (caller). Differences from
    // SearchPass, all deliberate:
    //  - scope = per-tag block top-512 pooled to a global top retrieve
    //    instead of the full scan; stride is IGNORED (full tag scope, better
    //    coverage).
    //  - playback-neighborhood safety samples are added explicitly so margin
    //    arbitration always sees the hold candidate even if retrieval missed
    //    it.
    //  - candidates are scored in ascending sample order so strictly-less
    //    tie-breaks match the brute-force scan exactly.
    // Returns true when a valid winner was scored (result filled).
    bool IndexedSearchPass(
        const MotionMatchingDatabase& database,
        const TagPartitionCache& partition,
        const float* queryPose,
        const float* queryTrajectory,
        const float* poseScale,
        const float* trajectoryScale,
        const MotionMatchingSearchSettings& settings,
        int32 currentClipIndex,
        bool lockSwitch,
        const Array<String>& allowedTags,
        MotionMatchingResult& result)
    {
        result = MotionMatchingResult();

        const int32 poseFeatureSize = database.GetPoseFeatureSize();
        const int32 trajectoryFeatureSize = database.GetTrajectoryFeatureSize();
        const int32 sampleCount = database.GetSampleCount();
        const int32 clipCount = database.GetClipCount();
        if (sampleCount <= 0 || clipCount <= 0)
            return false;

        const Array<int32>& sampleClipIndices = database.GetSampleClipIndices();
        const Array<float>& sampleTimes = database.GetSampleTimes();
        const Array<float>& poseFeatures = database.GetPoseFeatures();
        const Array<float>& trajectoryFeatures = database.GetTrajectoryFeatures();

        // Scratch buffers (phase 4): no per-query allocation on this path.
        IndexedScratch& scratch = GetIndexedScratch();

        // Shared precompute (same gates as SearchPass).
        const float continuityWeight = Math::Max(settings.ContinuityWeight, 0.0f);
        const bool continuityApplies = currentClipIndex >= 0 && currentClipIndex < clipCount;
        const bool useExclusion = continuityApplies && settings.CurrentTime >= 0.0f &&
            settings.SameClipExclusionWindow > 0.0f;
        const float exclusionWindow = Math::Max(settings.SameClipExclusionWindow, 0.0f);
        const bool useTimeTerms = continuityApplies && settings.ExpectedTime >= 0.0f;
        const float bias = Math::Max(settings.ContinuingBias, 0.0f);
        const float biasWindow = Math::Max(settings.ContinuingWindow, 0.0f);
        const float switchMargin = Math::Max(settings.SwitchMargin, 0.0f);
        const float loopPreference = Math::Max(settings.LoopPreference, 0.0f);
        const float twinBan = Math::Max(settings.TwinBanWindow, 0.0f);
        const float tailExclusion = Math::Max(settings.LoopTailExclusion, 0.0f);
        const bool useReselectBan = settings.ReselectClip >= 0 && settings.ReselectClip < clipCount &&
            settings.ReselectBanWindow > 0.0f;
        const float reselectWindow = Math::Max(settings.ReselectBanWindow, 0.0f);
        const Array<float>& clipLengths = database.GetClipLengths();
        const bool haveLengths = clipLengths.Count() == clipCount;
        const float trajectoryHorizon = GetTrajectoryHorizon(database);
        const bool excludeNonLoopTail = settings.ExcludeNonLoopTail;
        const bool wantLoop = settings.LoopFilter == MotionLoopFilter::LoopOnly;
        const bool wantOneShot = settings.LoopFilter == MotionLoopFilter::OneShotOnly;
        const int32 oneShotBand = settings.OneShotBand;

        // Retrieval: trajectory-coarse prefilter (replaces the 74-D tree
        // query � exact trees barely prune in high dimensions, measured 1.9x
        // only). Two cheap linear tiers over allowed-tag members:
        //   tier 1: exact pose+trajectory blocks with LIVE scales, per-tag
        //           top tier1Keep (same ordering as the rerank blocks, so no
        //           signal is ever dropped early � a trajectory-only tier was
        //           tried and REGRESSED recall to 1.7% on flat trajectories);
        //   tier 2: global top retrieve ? unchanged full scoring loop below
        //           applies every filter + term. TopKCandidates keeps its
        //           meaning (x6 finalists).
        int32 retrieve = settings.TopKCandidates > 0 ? settings.TopKCandidates * 6 : 192;
        if (retrieve < 16)
            retrieve = 16;
        if (retrieve > 512)
            retrieve = 512;
        // Tier-1 width tracks the finalist set (measured: TopK16 ? 96/288
        // gives sub-1ms scans with 0/1500 divergence; the linear scan costs
        // the same either way, this only sizes the selection).
        int32 tier1Keep = retrieve * 3;
        if (tier1Keep < 128)
            tier1Keep = 128;
        if (tier1Keep > 768)
            tier1Keep = 768;

        Array<CoarseEntry>& coarse = scratch.Coarse;
        coarse.Clear();
        Array<int32>& candidates = scratch.Candidates;
        candidates.Clear();
        const int32 tagCount = partition.TagNames.Count();
        int32 rangeBase[32];
        int32 rangeKept[32];
        int32 rangeCount = 0;
        // Perf phase 2: per-tag proportional tier-1 keep. A flat keep gives
        // the 590-sample tag the same 288 slots as the 22144-sample tag
        // (49% vs 1.3% coverage). Distribute the same total budget
        // (tier1Keep x allowed tags) by member count with a floor so tiny
        // tags still surface winners; the global top-retrieve below and the
        // full linear tier-1 scan are unchanged.
        int32 tagMembers[32];
        int32 tagTotal = 0;
        for (int32 t = 0; t < tagCount && rangeCount < 32; t++)
        {
            if (!TagAllowed(allowedTags, partition.TagNames[t]))
                continue;
            const int32 n = partition.SamplesByTag[t].Count();
            if (n <= 0)
                continue;
            tagMembers[rangeCount] = n;
            tagTotal += n;
            rangeCount++;
        }
        const int32 keepBudget = tier1Keep * rangeCount;
        int32 tagCursor = 0;
        for (int32 t = 0; t < tagCount && tagCursor < rangeCount; t++)
        {
            if (!TagAllowed(allowedTags, partition.TagNames[t]))
                continue;
            const Array<int32>& members = partition.SamplesByTag[t];
            const int32 n = members.Count();
            if (n <= 0)
                continue;
            // Proportional share, floor 16 (tiny tags keep a voice), cap n.
            int32 k = tagTotal > 0
                ? (int32)(((int64)keepBudget * tagMembers[tagCursor]) / tagTotal)
                : tier1Keep;
            if (k < 16)
                k = 16;
            if (k > n)
                k = n;
            tagCursor++;
            const int32 base = coarse.Count();
            // Excluded clips sink to MAX so they never take a retrieval slot
            // that a valid candidate needs (rerank drops them regardless).
            const bool coarseExclude = !settings.ExcludeTags.IsEmpty();
            for (int32 m = 0; m < n; m++)
            {
                const int32 s = members[m];
                if (coarseExclude)
                {
                    const int32 cs = sampleClipIndices[s];
                    if (cs >= 0 && cs < clipCount &&
                        TagExcluded(settings.ExcludeTags, database.GetClipTags(cs)))
                    {
                        CoarseEntry e;
                        e.Dist = MAX_float;
                        e.Sample = s;
                        coarse.Add(e);
                        continue;
                    }
                }
                const float* bakedPose = &poseFeatures[s * poseFeatureSize];
                const float* bakedTrajectory = &trajectoryFeatures[s * trajectoryFeatureSize];
                const float pose = PoseBlockCost(queryPose, bakedPose, poseScale, poseFeatureSize, MAX_float);
                CoarseEntry e;
                e.Dist = pose + TrajectoryBlockCost(
                    queryTrajectory, bakedTrajectory, trajectoryScale, trajectoryFeatureSize,
                    pose, MAX_float);
                e.Sample = s;
                coarse.Add(e);
            }
            CoarseEntry* begin = coarse.Get() + base;
            std::nth_element(begin, begin + k, begin + n,
                [](const CoarseEntry& a, const CoarseEntry& b) { return a.Dist < b.Dist; });
            rangeBase[tagCursor - 1] = base;
            rangeKept[tagCursor - 1] = k;
        }
        // Compact per-tag tops to the front (writes stay below reads: kept
        // totals never exceed the source range starts), then keep the global
        // top retrieve by the already-exact block score � no recompute.
        int32 pooled = 0;
        for (int32 r = 0; r < rangeCount; r++)
        {
            CoarseEntry* src = coarse.Get() + rangeBase[r];
            for (int32 i = 0; i < rangeKept[r]; i++)
                coarse[pooled++] = src[i];
        }
        if (pooled > 0)
        {
            CoarseEntry* pool = coarse.Get();
            int32 k = retrieve < pooled ? retrieve : pooled;
            std::nth_element(pool, pool + k, pool + pooled,
                [](const CoarseEntry& a, const CoarseEntry& b) { return a.Dist < b.Dist; });
            for (int32 i = 0; i < k; i++)
                candidates.Add(pool[i].Sample);
        }

        // Continuation safety set: the playback-relevant neighborhood on the
        // current clip is always exactly ranked, so the hold + margin
        // arbitration survive retrieval misses (rank data: same-clip twins
        // reordered by turn/foot terms the tree space cannot see). Window =
        // bias + twin-ban + exclusion reach: excluded/banned members are
        // still scored then filtered, so widening is harmless.
        if (useTimeTerms && currentClipIndex >= 0 && currentClipIndex < clipCount)
        {
            const float safetyWindow = biasWindow + twinBan + exclusionWindow;
            if (safetyWindow > 0.0f)
            {
                const Array<int32>& clipSampleCounts = database.GetClipSampleCounts();
                if (currentClipIndex < clipSampleCounts.Count())
                {
                    int32 start = 0;
                    for (int32 c = 0; c < currentClipIndex; c++)
                        start += clipSampleCounts[c];
                    const int32 end = start + clipSampleCounts[currentClipIndex];
                    for (int32 s = start; s < end && s < sampleCount; s++)
                    {
                        if (TimeDistance(database, currentClipIndex, settings.ExpectedTime, sampleTimes[s]) <= safetyWindow)
                            candidates.Add(s);
                    }
                }
            }
        }
        if (candidates.Count() == 0)
            return false;

        // Ascending sample order: strictly-less tie-breaks match SearchPass.
        // Dedup in place (no resize: the tail past uniqueCount is ignored).
        std::sort(candidates.Get(), candidates.Get() + candidates.Count());
        int32 uniqueCount = 0;
        for (int32 i = 0; i < candidates.Count(); i++)
        {
            if (i == 0 || candidates[i] != candidates[i - 1])
                candidates[uniqueCount++] = candidates[i];
        }

        float bestCost = MAX_float;
        float secondBestCost = MAX_float;
        float bestPoseCost = MAX_float;
        float bestTrajectoryCost = MAX_float;
        float bestContinuityCost = 0.0f;
        float bestLoopCost = 0.0f;
        int32 bestSample = -1;
        float bestContinueCost = MAX_float;
        float bestContinuePoseCost = 0.0f;
        float bestContinueTrajectoryCost = 0.0f;
        float bestContinueContinuityCost = 0.0f;
        float bestContinueLoopCost = 0.0f;
        int32 bestContinueSample = -1;
        int32 cachedLoopClip = -1;
        bool cachedLoopFlag = false;
        float cachedTurnAngle = 0.0f;
        int32 cachedSpeedBand = 0;
        float cachedTailStart = -1.0f;
        // Tags resolve on the clip-change tick below (candidates arrive in
        // ascending sample order, still clip-grouped); empty scope skips it.
        const bool checkTags = !allowedTags.IsEmpty() || !settings.ExcludeTags.IsEmpty();
        bool cachedTagOk = true;
        int32 loopScoredCount = 0;
        int32 oneShotScoredCount = 0;

        for (int32 ci = 0; ci < uniqueCount; ci++)
        {
            const int32 sampleIndex = candidates[ci];
            const int32 clipIndex = sampleClipIndices[sampleIndex];
            if (clipIndex < 0 || clipIndex >= clipCount)
                continue;
            if (clipIndex != cachedLoopClip)
            {
                cachedLoopClip = clipIndex;
                cachedLoopFlag = database.GetClipLoop(clipIndex);
                cachedTurnAngle = database.GetClipTurnAngle(clipIndex);
                cachedSpeedBand = database.GetClipSpeedBand(clipIndex);
                const Array<String>& clipTagSet = database.GetClipTags(clipIndex);
                cachedTagOk = !checkTags || (TagAllowed(allowedTags, clipTagSet) && !TagExcluded(settings.ExcludeTags, clipTagSet));
                cachedTailStart = -1.0f;
                if (tailExclusion > 0.0f && !cachedLoopFlag && haveLengths &&
                    clipLengths[clipIndex] > tailExclusion)
                    cachedTailStart = clipLengths[clipIndex] - tailExclusion;
            }
            if ((wantLoop && !cachedLoopFlag) || (wantOneShot && cachedLoopFlag))
                continue;
            if (oneShotBand != 0 && !cachedLoopFlag && cachedSpeedBand != 0 &&
                cachedSpeedBand != oneShotBand)
                continue;
            if (lockSwitch && clipIndex != currentClipIndex)
                continue;
            if (!cachedTagOk)
                continue;
            if (cachedTailStart >= 0.0f && sampleTimes[sampleIndex] > cachedTailStart)
                continue;
            if (excludeNonLoopTail && haveLengths &&
                MotionMatchingSampler::IsNonLoopFutureOutside(
                    sampleTimes[sampleIndex], trajectoryHorizon, clipLengths[clipIndex], cachedLoopFlag))
                continue;
            if (useReselectBan && clipIndex == settings.ReselectClip &&
                TimeDistance(database, clipIndex, settings.ReselectSampleTime, sampleTimes[sampleIndex]) <= reselectWindow)
                continue;
            const bool isContinuation = IsContinuationSample(
                database, settings, useTimeTerms, currentClipIndex, clipIndex,
                sampleTimes[sampleIndex], biasWindow);
            if (useTimeTerms && twinBan > 0.0f && clipIndex == currentClipIndex && !isContinuation &&
                TimeDistance(database, clipIndex, settings.ExpectedTime, sampleTimes[sampleIndex]) <= biasWindow + twinBan)
                continue;
            if (useExclusion && clipIndex == currentClipIndex && !isContinuation)
            {
                if (TimeDistance(database, clipIndex, settings.CurrentTime, sampleTimes[sampleIndex]) <= exclusionWindow)
                    continue;
            }

            const float pruneLimit = secondBestCost +
                (isContinuation ? (bias > 0.0f ? bias : 0.0f) + switchMargin : 0.0f) +
                (cachedLoopFlag && loopPreference > 0.0f ? loopPreference : 0.0f);
            const float* bakedPose = &poseFeatures[sampleIndex * poseFeatureSize];
            const float* bakedTrajectory = &trajectoryFeatures[sampleIndex * trajectoryFeatureSize];
            const float poseCost = PoseBlockCost(
                queryPose, bakedPose, poseScale, poseFeatureSize, pruneLimit);
            if (poseCost > pruneLimit)
                continue;
            const float trajectoryCost = TrajectoryBlockCost(
                queryTrajectory, bakedTrajectory, trajectoryScale, trajectoryFeatureSize,
                poseCost, pruneLimit);
            if (poseCost + trajectoryCost > pruneLimit)
                continue;

            float continuityCost = ContinuityTerm(
                continuityApplies, clipIndex != currentClipIndex, isContinuation,
                continuityWeight, bias);
            if (settings.TurnPenaltyWeight > 0.0f)
                continuityCost += TurnPenalty(cachedTurnAngle, settings.QueryYawRate,
                    settings.TurnPenaltyWeight, settings.TurnPenaltyYawScale);
            continuityCost += FootLockCost(database, bakedPose, poseFeatureSize, settings);
            const float loopCost = LoopPreferenceTerm(cachedLoopFlag, loopPreference);
            const float cost = poseCost + trajectoryCost + continuityCost + loopCost;

            if (cachedLoopFlag)
                loopScoredCount++;
            else
                oneShotScoredCount++;

            if (cost < bestCost)
            {
                secondBestCost = bestCost;
                bestCost = cost;
                bestSample = sampleIndex;
                bestPoseCost = poseCost;
                bestTrajectoryCost = trajectoryCost;
                bestContinuityCost = continuityCost;
                bestLoopCost = loopCost;
            }
            else if (cost < secondBestCost)
            {
                secondBestCost = cost;
            }
            if (isContinuation && cost < bestContinueCost)
            {
                bestContinueCost = cost;
                bestContinueSample = sampleIndex;
                bestContinuePoseCost = poseCost;
                bestContinueTrajectoryCost = trajectoryCost;
                bestContinueContinuityCost = continuityCost;
                bestContinueLoopCost = loopCost;
            }
        }

        if (bestSample >= 0 && bestContinueSample >= 0 &&
            sampleClipIndices[bestSample] != currentClipIndex &&
            switchMargin > 0.0f && bestCost + switchMargin >= bestContinueCost)
        {
            secondBestCost = bestCost;
            bestCost = bestContinueCost;
            bestSample = bestContinueSample;
            bestPoseCost = bestContinuePoseCost;
            bestTrajectoryCost = bestContinueTrajectoryCost;
            bestContinuityCost = bestContinueContinuityCost;
            bestLoopCost = bestContinueLoopCost;
        }

        if (bestSample < 0)
            return false;

        result.IsValid = true;
        result.SampleIndex = bestSample;
        result.ClipIndex = sampleClipIndices[bestSample];
        result.SampleTime = sampleTimes[bestSample];
        result.Cost = bestCost;
        result.SecondBestCost = secondBestCost;
        result.PoseCost = bestPoseCost;
        result.TrajectoryCost = bestTrajectoryCost;
        result.ContinuityCost = bestContinuityCost;
        result.LoopCost = bestLoopCost;
        if (settings.CollectClassDiagnostics)
        {
            ComputeClassBests(
                database,
                queryPose,
                queryTrajectory,
                poseScale,
                trajectoryScale,
                allowedTags,
                settings.ExcludeTags,
                settings.LoopFilter,
                settings.OneShotBand,
                result.BestLoopCost,
                result.BestLoopSample,
                result.BestLoopClip,
                result.LoopTotalCount,
                result.BestOneShotCost,
                result.BestOneShotSample,
                result.BestOneShotClip,
                result.OneShotTotalCount);
        }
        result.LoopScoredCount = loopScoredCount;
        result.OneShotScoredCount = oneShotScoredCount;
        return true;
    }
}

void MotionMatchingSearch::FindTopCandidates(
    MotionMatchingDatabase* database,
    const Array<float>& queryPose,
    const Array<float>& queryTrajectory,
    const MotionMatchingSearchSettings& settings,
    int32 currentClipIndex,
    const Array<String>& allowedTags,
    int32 topCount,
    Array<float>& packed,
    String& error)
{
    packed.Clear();
    error = String::Empty;

    if (database == nullptr)
    {
        error = TEXT("Motion matching top-N failed: database is not loaded.");
        return;
    }
    if (!IsReadyForQuery(*database, error))
        return;

    // Phase 0: build (or reuse) the tag-partition stats. Return value unused:
    // the hot pass still scans everything; this only counts + logs.
    (void)EnsureTagPartition(*database);

    const int32 poseFeatureSize = database->GetPoseFeatureSize();
    const int32 trajectoryFeatureSize = database->GetTrajectoryFeatureSize();
    if (queryPose.Count() != poseFeatureSize || queryTrajectory.Count() != trajectoryFeatureSize)
    {
        error = TEXT("Motion matching top-N failed: query dimensions do not match schema.");
        return;
    }
    for (int32 i = 0; i < queryPose.Count(); i++)
    {
        if (!std::isfinite(queryPose[i]))
        {
            error = TEXT("Motion matching top-N failed: query pose contains a non-finite value.");
            return;
        }
    }
    for (int32 i = 0; i < trajectoryFeatureSize; i++)
    {
        if (!std::isfinite(queryTrajectory[i]))
        {
            error = TEXT("Motion matching top-N failed: query trajectory contains a non-finite value.");
            return;
        }
    }

    // N1b: raw query + one per-dim scale pass (same pipeline as FindBest).
    float stackPoseScale[PoseSearchSchema::GetPoseFeatureSize()];
    float stackTrajectoryScale[PoseSearchSchema::GetTrajectoryFeatureSize()];
    Array<float> heapPoseScale, heapTrajectoryScale;
    float* poseScale = stackPoseScale;
    float* trajectoryScale = stackTrajectoryScale;
    if (poseFeatureSize > PoseSearchSchema::GetPoseFeatureSize())
    {
        heapPoseScale.Resize(poseFeatureSize);
        poseScale = heapPoseScale.Get();
    }
    if (trajectoryFeatureSize > PoseSearchSchema::GetTrajectoryFeatureSize())
    {
        heapTrajectoryScale.Resize(trajectoryFeatureSize);
        trajectoryScale = heapTrajectoryScale.Get();
    }
    BuildQueryScales(*database, settings, poseScale, trajectoryScale);

    const int32 n = Math::Clamp(topCount, 1, 32);
    constexpr int32 CandidateStride = 10;
    // Fixed stack covers the max clamp above; larger requests are clamped, so
    // no heap path is needed here.
    float top[32 * CandidateStride];
    for (int32 i = 0; i < 32 * CandidateStride; i++)
        top[i] = MAX_float;
    int32 collected = 0;

    const int32 clipCount = database->GetClipCount();
    const int32 sampleCount = database->GetSampleCount();
    const Array<int32>& sampleClipIndices = database->GetSampleClipIndices();
    const Array<float>& sampleTimes = database->GetSampleTimes();
    const Array<float>& poseFeatures = database->GetPoseFeatures();
    const Array<float>& trajectoryFeatures = database->GetTrajectoryFeatures();

    const float continuityWeight = Math::Max(settings.ContinuityWeight, 0.0f);
    const bool continuityApplies = currentClipIndex >= 0 && currentClipIndex < clipCount;
    const bool useExclusion = continuityApplies && settings.CurrentTime >= 0.0f &&
        settings.SameClipExclusionWindow > 0.0f;
    const float exclusionWindow = Math::Max(settings.SameClipExclusionWindow, 0.0f);
    const bool useTimeTerms = continuityApplies && settings.ExpectedTime >= 0.0f;
    const float bias = Math::Max(settings.ContinuingBias, 0.0f);
    const float biasWindow = Math::Max(settings.ContinuingWindow, 0.0f);
    const float loopPreference = Math::Max(settings.LoopPreference, 0.0f);
    const bool wantLoop = settings.LoopFilter == MotionLoopFilter::LoopOnly;
    const bool wantOneShot = settings.LoopFilter == MotionLoopFilter::OneShotOnly;
    // Same Wave 1 bans as the hot pass (top-N must rank what runtime plays).
    const float twinBan = Math::Max(settings.TwinBanWindow, 0.0f);
    const float tailExclusion = Math::Max(settings.LoopTailExclusion, 0.0f);
    const bool useReselectBan = settings.ReselectClip >= 0 && settings.ReselectClip < clipCount &&
        settings.ReselectBanWindow > 0.0f;
    const float reselectWindow = Math::Max(settings.ReselectBanWindow, 0.0f);
    const Array<float>& clipLengths = database->GetClipLengths();
    const bool haveLengths = clipLengths.Count() == clipCount;
    // Same structural validity cut as the hot pass (S1 �7.2/�18.6): top-N must
    // rank what runtime could actually play, so a non-loop clamped-future
    // candidate is excluded here too.
    const float trajectoryHorizon = GetTrajectoryHorizon(*database);
    const bool excludeNonLoopTail = settings.ExcludeNonLoopTail;
    const int32 oneShotBand = settings.OneShotBand;

    int32 cachedLoopClip = -1;
    bool cachedLoopFlag = false;
    float cachedTurnAngle = 0.0f;
    int32 cachedSpeedBand = 0;
    float cachedTailStart = -1.0f;
    // Tooling path too: tags resolve once per clip, not per sample.
    const bool checkTags = !allowedTags.IsEmpty();
    bool cachedTagOk = true;
    for (int32 sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
    {
        const int32 clipIndex = sampleClipIndices[sampleIndex];
        if (clipIndex < 0 || clipIndex >= clipCount)
            continue;
        if (clipIndex != cachedLoopClip)
        {
            cachedLoopClip = clipIndex;
            cachedLoopFlag = database->GetClipLoop(clipIndex);
            cachedTurnAngle = database->GetClipTurnAngle(clipIndex);
            cachedSpeedBand = database->GetClipSpeedBand(clipIndex);
            const Array<String>& clipTagSet = database->GetClipTags(clipIndex);
            cachedTagOk = !checkTags || (TagAllowed(allowedTags, clipTagSet) && !TagExcluded(settings.ExcludeTags, clipTagSet));
            cachedTailStart = -1.0f;
            if (tailExclusion > 0.0f && !cachedLoopFlag && haveLengths &&
                clipLengths[clipIndex] > tailExclusion)
                cachedTailStart = clipLengths[clipIndex] - tailExclusion;
        }
        if ((wantLoop && !cachedLoopFlag) || (wantOneShot && cachedLoopFlag))
            continue;
        if (oneShotBand != 0 && !cachedLoopFlag && cachedSpeedBand != 0 &&
            cachedSpeedBand != oneShotBand)
            continue;
        if (!cachedTagOk)
            continue;
        if (cachedTailStart >= 0.0f && sampleTimes[sampleIndex] > cachedTailStart)
            continue;
        if (excludeNonLoopTail && haveLengths &&
            MotionMatchingSampler::IsNonLoopFutureOutside(
                sampleTimes[sampleIndex], trajectoryHorizon, clipLengths[clipIndex], cachedLoopFlag))
            continue;
        if (useReselectBan && clipIndex == settings.ReselectClip &&
            TimeDistance(*database, clipIndex, settings.ReselectSampleTime, sampleTimes[sampleIndex]) <= reselectWindow)
            continue;
        const bool isContinuation = IsContinuationSample(
            *database, settings, useTimeTerms, currentClipIndex, clipIndex,
            sampleTimes[sampleIndex], biasWindow);
        if (useExclusion && clipIndex == currentClipIndex && !isContinuation)
        {
            if (TimeDistance(*database, clipIndex, settings.CurrentTime, sampleTimes[sampleIndex]) <= exclusionWindow)
                continue;
        }
        if (useTimeTerms && twinBan > 0.0f && clipIndex == currentClipIndex && !isContinuation &&
            TimeDistance(*database, clipIndex, settings.ExpectedTime, sampleTimes[sampleIndex]) <= biasWindow + twinBan)
            continue;

        // Exact scan: no pruning here (tooling path, ~1ms for 68k samples),
        // so the shared term helpers run with an infinite prune bound.
        const float* bakedPose = &poseFeatures[sampleIndex * poseFeatureSize];
        const float* bakedTrajectory = &trajectoryFeatures[sampleIndex * trajectoryFeatureSize];
        const float poseCost = PoseBlockCost(
            queryPose.Get(), bakedPose, poseScale, poseFeatureSize, MAX_float);
        const float trajectoryCost = TrajectoryBlockCost(
            queryTrajectory.Get(), bakedTrajectory, trajectoryScale, trajectoryFeatureSize,
            poseCost, MAX_float);
        float continuityCost = ContinuityTerm(
            continuityApplies, clipIndex != currentClipIndex, isContinuation,
            continuityWeight, bias);
        if (settings.TurnPenaltyWeight > 0.0f)
            continuityCost += TurnPenalty(cachedTurnAngle, settings.QueryYawRate,
                settings.TurnPenaltyWeight, settings.TurnPenaltyYawScale);
        continuityCost += FootLockCost(*database, bakedPose, poseFeatureSize, settings);
        const float loopCost = LoopPreferenceTerm(cachedLoopFlag, loopPreference);
        const float cost = poseCost + trajectoryCost + continuityCost + loopCost;

        // Sorted insertion, strictly-less: ties keep the lower sample index
        // because the scan itself is ascending.
        int32 slot = collected < n ? collected : n;
        while (slot > 0 && cost < top[(slot - 1) * CandidateStride + 3])
            slot--;
        if (slot >= n)
            continue;
        if (collected < n)
            collected++;
        for (int32 k = collected - 1; k > slot; k--)
        {
            for (int32 f = 0; f < CandidateStride; f++)
                top[k * CandidateStride + f] = top[(k - 1) * CandidateStride + f];
        }
        top[slot * CandidateStride + 0] = (float)sampleIndex;
        top[slot * CandidateStride + 1] = (float)clipIndex;
        top[slot * CandidateStride + 2] = sampleTimes[sampleIndex];
        top[slot * CandidateStride + 3] = cost;
        top[slot * CandidateStride + 4] = poseCost;
        top[slot * CandidateStride + 5] = trajectoryCost;
        top[slot * CandidateStride + 6] = continuityCost;
        top[slot * CandidateStride + 7] = loopCost;
        top[slot * CandidateStride + 8] = isContinuation ? 1.0f : 0.0f;
        top[slot * CandidateStride + 9] = cachedLoopFlag ? 1.0f : 0.0f;
    }

    packed.Resize(collected * CandidateStride);
    for (int32 i = 0; i < collected * CandidateStride; i++)
        packed[i] = top[i];
    if (collected == 0)
        error = TEXT("Motion matching top-N found no valid candidate. Check tag filters.");
}

void MotionMatchingSearch::FindBest(
    MotionMatchingDatabase* database,
    const Array<float>& queryPose,
    const Array<float>& queryTrajectory,
    const MotionMatchingSearchSettings& settings,
    int32 currentClipIndex,
    float timeSinceSwitch,
    const Array<String>& allowedTags,
    MotionMatchingResult& result,
    String& error)
{
    result = MotionMatchingResult();
    error = String::Empty;

    if (database == nullptr)
    {
        error = TEXT("Motion matching search failed: database is not loaded.");
        return;
    }
    if (!IsReadyForQuery(*database, error))
        return;

    // Phase 0+1: tag-partition stats + per-tag KD-trees (built once per DB).
    const TagPartitionCache& partition = EnsureTagPartition(*database);

    const int32 poseFeatureSize = database->GetPoseFeatureSize();
    const int32 trajectoryFeatureSize = database->GetTrajectoryFeatureSize();
    if (queryPose.Count() != poseFeatureSize || queryTrajectory.Count() != trajectoryFeatureSize)
    {
        error = String::Format(
            TEXT("Motion matching search failed: query dimensions (pose={}, trajectory={}) do not match schema ({}, {})."),
            queryPose.Count(),
            queryTrajectory.Count(),
            poseFeatureSize,
            trajectoryFeatureSize);
        return;
    }
    for (int32 i = 0; i < queryPose.Count(); i++)
    {
        if (!std::isfinite(queryPose[i]))
        {
            error = TEXT("Motion matching search failed: query pose contains a non-finite value.");
            return;
        }
    }
    for (int32 i = 0; i < queryTrajectory.Count(); i++)
    {
        if (!std::isfinite(queryTrajectory[i]))
        {
            error = TEXT("Motion matching search failed: query trajectory contains a non-finite value.");
            return;
        }
    }
    if (!std::isfinite(settings.PoseWeight) || !std::isfinite(settings.TrajectoryWeight) ||
        !std::isfinite(settings.ContinuityWeight) || !std::isfinite(settings.MinSwitchTime) ||
        !std::isfinite(settings.CurrentTime) || !std::isfinite(settings.SameClipExclusionWindow) ||
        !std::isfinite(settings.ExpectedTime) || !std::isfinite(settings.ContinuingBias) ||
        !std::isfinite(settings.ContinuingWindow) || !std::isfinite(settings.SwitchMargin) ||
        !std::isfinite(settings.LoopPreference) || !std::isfinite(settings.QueryYawRate) ||
        !std::isfinite(settings.ReselectSampleTime) || !std::isfinite(settings.ReselectBanWindow) ||
        !std::isfinite(settings.TwinBanWindow) || !std::isfinite(settings.LoopTailExclusion))
    {
        error = TEXT("Motion matching search failed: settings contain a non-finite value.");
        return;
    }
    if (settings.LoopFilter != MotionLoopFilter::Any &&
        settings.LoopFilter != MotionLoopFilter::LoopOnly &&
        settings.LoopFilter != MotionLoopFilter::OneShotOnly)
    {
        error = TEXT("Motion matching search failed: unknown loop filter.");
        return;
    }

    const int32 sampleCount = database->GetSampleCount();
    // Brute-force reference scans everything by default (MaxCandidates <= 0).
    // A positive budget strides the scan; stride > 1 is an explicit quality
    // trade, never the default.
    int32 stride = 1;
    if (settings.MaxCandidates > 0 && settings.MaxCandidates < sampleCount)
        stride = (sampleCount + settings.MaxCandidates - 1) / settings.MaxCandidates;

    const int32 clipCount = database->GetClipCount();
    const bool hasCurrent = currentClipIndex >= 0 && currentClipIndex < clipCount;
    const bool lockSwitch = hasCurrent && timeSinceSwitch < settings.MinSwitchTime;

    // N1b rev 3: the query stays raw. Build one per-dim scale array
    // (sqrt(sum-normalized weight)/deviation) ONCE; the hot loop then squares
    // scale*(raw query - raw baked). Fixed stack caps cover the v3 layout
    // (44 + 30); larger future schemas fall back to heap. Shared with the
    // top-N/tooling path.
    float stackPoseScale[PoseSearchSchema::GetPoseFeatureSize()];
    float stackTrajectoryScale[PoseSearchSchema::GetTrajectoryFeatureSize()];
    Array<float> heapPoseScale, heapTrajectoryScale;
    float* poseScale = stackPoseScale;
    float* trajectoryScale = stackTrajectoryScale;
    if (poseFeatureSize > PoseSearchSchema::GetPoseFeatureSize())
    {
        heapPoseScale.Resize(poseFeatureSize);
        poseScale = heapPoseScale.Get();
    }
    if (trajectoryFeatureSize > PoseSearchSchema::GetTrajectoryFeatureSize())
    {
        heapTrajectoryScale.Resize(trajectoryFeatureSize);
        trajectoryScale = heapTrajectoryScale.Get();
    }
    BuildQueryScales(*database, settings, poseScale, trajectoryScale);
    const float* rawPose = queryPose.Get();
    const float* rawTrajectory = queryTrajectory.Get();

    // Phase 1: indexed retrieval + exact rerank behind a default-off flag.
    // Falls back to the brute-force reference whenever it cannot score a
    // valid winner (missing tag tree, filters exhausting top-K, ...).
    // Phase 2: with VerifyIndexedSearch the reference ALSO runs, divergence
    // is counted to the log, and the BRUTE-FORCE winner is kept (behavior
    // unchanged during verify).
    MotionMatchingResult indexedResult;
    const bool wantVerify = settings.UseIndexedSearch && settings.VerifyIndexedSearch;
    bool indexedOk = false;
    double indexedMs = 0.0;
    double bruteMs = 0.0;
    if (settings.UseIndexedSearch)
    {
        const auto indexedStart = std::chrono::high_resolution_clock::now();
        indexedOk = IndexedSearchPass(
            *database,
            partition,
            rawPose,
            rawTrajectory,
            poseScale,
            trajectoryScale,
            settings,
            currentClipIndex,
            lockSwitch,
            allowedTags,
            indexedResult);
        if (wantVerify)
            indexedMs = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - indexedStart).count();
    }
    if (!indexedOk || wantVerify)
    {
        // Verify compares against the EXACT reference: stride would hide the
        // global best from brute force (by design) and fake divergence, so
        // the reference always scans unstrided here. The live strided path is
        // unaffected (wantVerify is false outside audits).
        const auto bruteStart = std::chrono::high_resolution_clock::now();
        SearchPass(
            *database,
            rawPose,
            rawTrajectory,
            poseScale,
            trajectoryScale,
            settings,
            currentClipIndex,
            lockSwitch,
            allowedTags,
            wantVerify ? 1 : stride,
            result);
        if (wantVerify)
            bruteMs = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - bruteStart).count();
    }
    else
    {
        result = indexedResult;
    }
    if (wantVerify)
        VerifyIndexedResult(*database, indexedOk, indexedResult, result, indexedMs, bruteMs);

    if (!result.IsValid && lockSwitch)
    {
        // Continuity lock filtered out every candidate (e.g. the allowed tags
        // exclude the current clip). Forced-interruption semantics: search
        // again with switching permitted rather than returning nothing.
        SearchPass(
            *database,
            rawPose,
            rawTrajectory,
            poseScale,
            trajectoryScale,
            settings,
            currentClipIndex,
            false,
            allowedTags,
            stride,
            result);
    }

    if (!result.IsValid)
        error = TEXT("Motion matching search found no valid candidate. Check tag filters and continuity settings.");
}

bool MotionMatchingSearch::SampleCost(
    MotionMatchingDatabase* database,
    const Array<float>& queryPose,
    const Array<float>& queryTrajectory,
    int32 sampleIndex,
    float poseWeight,
    float trajectoryWeight,
    float queryYawRate,
    bool applyTurnPenalty,
    float turnPenaltyWeight,
    float turnPenaltyYawScale,
    float& cost,
    String& error)
{
    cost = 0.0f;
    error = String::Empty;

    if (database == nullptr)
    {
        error = TEXT("Motion matching sample cost failed: database is not loaded.");
        return false;
    }
    if (!IsReadyForQuery(*database, error))
        return false;

    const int32 poseFeatureSize = database->GetPoseFeatureSize();
    const int32 trajectoryFeatureSize = database->GetTrajectoryFeatureSize();
    if (queryPose.Count() != poseFeatureSize || queryTrajectory.Count() != trajectoryFeatureSize)
    {
        error = TEXT("Motion matching sample cost failed: query dimensions do not match schema.");
        return false;
    }
    if (sampleIndex < 0 || sampleIndex >= database->GetSampleCount())
    {
        error = TEXT("Motion matching sample cost failed: sample index out of range.");
        return false;
    }
    for (int32 i = 0; i < queryPose.Count(); i++)
    {
        if (!std::isfinite(queryPose[i]))
        {
            error = TEXT("Motion matching sample cost failed: query pose contains a non-finite value.");
            return false;
        }
    }
    for (int32 i = 0; i < queryTrajectory.Count(); i++)
    {
        if (!std::isfinite(queryTrajectory[i]))
        {
            error = TEXT("Motion matching sample cost failed: query trajectory contains a non-finite value.");
            return false;
        }
    }

    // S1 term parity: the fast-path estimate uses the same FullSampleCost term
    // definitions as the hot pass, with no playback context (continuity/loop/
    // foot disabled exactly as before: raw pose + trajectory + straightness).
    // P2: turn prior weights ride explicit params (trace-recorded tuning).
    MotionMatchingSearchSettings costSettings;
    costSettings.PoseWeight = poseWeight;
    costSettings.TrajectoryWeight = trajectoryWeight;
    costSettings.QueryYawRate = queryYawRate;
    costSettings.TurnPenaltyWeight = turnPenaltyWeight;
    costSettings.TurnPenaltyYawScale = turnPenaltyYawScale;
    costSettings.ContinuityWeight = 0.0f;
    costSettings.CurrentTime = -1.0f;
    costSettings.ExpectedTime = -1.0f;
    costSettings.ContinuingBias = 0.0f;
    costSettings.LoopPreference = 0.0f;
    costSettings.FootLockL = 0.0f;
    costSettings.FootLockR = 0.0f;

    // N1b rev 3: raw query; FullSampleCost builds its own scale pass.
    cost = FullSampleCost(
        *database, costSettings, queryPose.Get(), queryTrajectory.Get(),
        -1, sampleIndex, applyTurnPenalty);
    return true;
}

bool MotionMatchingSearch::RunSelfTest(
    MotionMatchingDatabase* database,
    String& log)
{
    log = String::Empty;
    if (database == nullptr || database->WaitForLoaded())
    {
        log = TEXT("Self-test failed: database is not loaded.");
        return false;
    }
    if (!database->IsBaked() || !database->ValidateData())
    {
        log = TEXT("Self-test failed: database is not baked or not valid.");
        return false;
    }

    const int32 poseFeatureSize = database->GetPoseFeatureSize();
    const int32 trajectoryFeatureSize = database->GetTrajectoryFeatureSize();
    const int32 sampleCount = database->GetSampleCount();
    const Array<float>& poseFeatures = database->GetPoseFeatures();
    const Array<float>& trajectoryFeatures = database->GetTrajectoryFeatures();
    const Array<float>& poseDeviations = database->GetPoseFeatureDeviations();
    const Array<float>& trajectoryDeviations = database->GetTrajectoryFeatureDeviations();

    // Helper: an exact query is a straight copy of the RAW baked sample
    // (rev 3 keeps baked values raw; no denormalization, no mean/deviation).
    auto buildExactQuery = [&](int32 source, Array<float>& queryPose, Array<float>& queryTrajectory)
    {
        queryPose.Resize(poseFeatureSize);
        queryTrajectory.Resize(trajectoryFeatureSize);
        const float* bakedPose = &poseFeatures[source * poseFeatureSize];
        const float* bakedTrajectory = &trajectoryFeatures[source * trajectoryFeatureSize];
        for (int32 i = 0; i < poseFeatureSize; i++)
            queryPose[i] = bakedPose[i];
        for (int32 i = 0; i < trajectoryFeatureSize; i++)
            queryTrajectory[i] = bakedTrajectory[i];
    };

    MotionMatchingSearchSettings settings;
    settings.PoseWeight = database->GetSchema().PoseWeight;
    settings.TrajectoryWeight = database->GetSchema().TrajectoryWeight;
    settings.ContinuityWeight = 0.0f;
    settings.MinSwitchTime = 0.0f;
    // Explicit self-test exemption from the Wave 1 runtime bans: exact-query
    // probes must see the full database (a banned tail sample could never
    // re-verify itself). The runtime policy opts in per query instead.
    settings.ReselectClip = -1;
    settings.ReselectSampleTime = 0.0f;
    settings.ReselectBanWindow = 0.0f;
    settings.TwinBanWindow = 0.0f;
    settings.LoopTailExclusion = 0.0f;
    // S1 �18.6: exact-query probes copy raw baked samples (including the
    // clamped non-loop tail) and must be able to re-find them, so the
    // structural runtime validity cut is disabled here and covered directly
    // by Phase L instead.
    settings.ExcludeNonLoopTail = false;
    const Array<String> noTags;

    int32 passed = 0;
    int32 tested = 0;
    float worstExactCost = 0.0f;
    int32 worstExactSample = -1;
    float minPoseDeviation = MAX_float;
    float minTrajectoryDeviation = MAX_float;
    for (int32 i = 0; i < poseFeatureSize; i++)
        minPoseDeviation = Math::Min(minPoseDeviation, poseDeviations[i]);
    for (int32 i = 0; i < trajectoryFeatureSize; i++)
        minTrajectoryDeviation = Math::Min(minTrajectoryDeviation, trajectoryDeviations[i]);
    log += String::Format(
        TEXT("Min stored deviation: pose={:.8f}, trajectory={:.8f} (raw values; exact-query cost is 0 regardless).\n"),
        minPoseDeviation,
        minTrajectoryDeviation);
    const Array<int32>& sampleClipIndices = database->GetSampleClipIndices();
    const Array<int32>& clipSampleCounts = database->GetClipSampleCounts();
    const Array<String>& tagNames = database->GetAnimationTags();
    const int32 clipCount = database->GetClipCount();
    const auto startTime = std::chrono::high_resolution_clock::now();

    Array<float> queryPose;
    Array<float> queryTrajectory;

    // Exact-query verdict. The winner must be within eps of the PROBE
    // itself (near-optimal), not of zero: turn probes can never cost ~0 by
    // design (straightness prior), while straight probes keep the old
    // behavior (their own penalty is 0). On mismatch, rescan everything
    // before the winner with the SAME penalty-aware math and require
    // strictly greater cost � otherwise the earlier sample should have won
    // under strictly-less comparison.
    int32 tieCount = 0;
    auto checkExact = [&](int32 source, const MotionMatchingResult& result, const String& error, const String& phaseLabel) -> bool
    {
        tested++;
        const int32 sourceClip = (source >= 0 && source < sampleClipIndices.Count()) ? sampleClipIndices[source] : -1;
        const float sourcePenalty = TurnPenalty(database->GetClipTurnAngle(sourceClip), settings.QueryYawRate,
            settings.TurnPenaltyWeight, settings.TurnPenaltyYawScale);
        if (!error.IsEmpty() || !result.IsValid || result.Cost > sourcePenalty + SelfTestMaxCost)
        {
            log += String::Format(
                TEXT("{} FAIL: sample {} (winner={}, cost={:.6f}, err='{}').\n"),
                phaseLabel,
                source,
                result.SampleIndex,
                result.Cost,
                error);
            return false;
        }
        if (result.SampleIndex != source)
        {
            tieCount++;
            for (int32 s = 0; s < result.SampleIndex; s++)
            {
                const float earlier = FullSampleCost(*database, queryPose, queryTrajectory, settings, -1, s, true);
                if (earlier <= result.Cost)
                {
                    log += String::Format(
                        TEXT("{} TIE-BREAK BROKEN: sample {} won by {} but earlier sample {} costs {:.9f} <= {:.9f}.\n"),
                        phaseLabel,
                        source,
                        result.SampleIndex,
                        s,
                        earlier,
                        result.Cost);
                    return false;
                }
            }
        }
        passed++;
        if (result.Cost > worstExactCost)
        {
            worstExactCost = result.Cost;
            worstExactSample = source;
        }
        return true;
    };

    // Phase A: clip boundaries. First/last samples exercise loop wrap-around
    // trajectory and clamped endpoint velocities � the riskiest baked data.
    // Only failures are logged individually.
    int32 phaseATested = 0;
    int32 phaseAPassed = 0;
    int32 sampleCursor = 0;
    for (int32 clipIndex = 0; clipIndex < clipCount; clipIndex++)
    {
        const int32 count = clipSampleCounts[clipIndex];
        if (count <= 0)
        {
            continue;
        }
        const int32 edges[2] = { sampleCursor, sampleCursor + count - 1 };
        for (int32 e = 0; e < 2; e++)
        {
            const int32 source = edges[e];
            buildExactQuery(source, queryPose, queryTrajectory);
            MotionMatchingResult result;
            String error;
            FindBest(database, queryPose, queryTrajectory, settings, -1, MAX_float, noTags, result, error);
            phaseATested++;
            if (checkExact(source, result, error, TEXT("Boundary")))
                phaseAPassed++;
        }
        sampleCursor += count;
    }
    log += String::Format(
        TEXT("Phase A (clip boundaries): {}/{} passed, worst exact cost={:.6f} at sample {}.\n"),
        phaseAPassed,
        phaseATested,
        worstExactCost,
        worstExactSample);

    // Phase B: strided spread over the whole database.
    int32 phaseBTested = 0;
    int32 phaseBPassed = 0;
    for (int32 source = 0; source < sampleCount; source += 997)
    {
        buildExactQuery(source, queryPose, queryTrajectory);
        MotionMatchingResult result;
        String error;
        FindBest(database, queryPose, queryTrajectory, settings, -1, MAX_float, noTags, result, error);
        phaseBTested++;
        if (checkExact(source, result, error, TEXT("Spread")))
            phaseBPassed++;
    }
    log += String::Format(TEXT("Phase B (spread): {}/{} passed.\n"), phaseBPassed, phaseBTested);

    // Phase C: policy behavior on spread samples � continuity lock keeps the
    // current clip, tag include/exclude filters steer the winner tag.
    MotionMatchingSearchSettings lockedSettings = settings;
    lockedSettings.ContinuityWeight = 1.0f;
    lockedSettings.MinSwitchTime = 10.0f;
    int32 phaseCTest = 0;
    int32 phaseCPass = 0;
    for (int32 source = 0; source < sampleCount && phaseCTest < 60; source += 9973)
    {
        const int32 sourceClip = sampleClipIndices[source];
        const Array<String> sourceTags = database->GetClipTags(sourceClip);
        const String sourceTag = MotionMatchingDatabase::JoinTags(sourceTags);
        buildExactQuery(source, queryPose, queryTrajectory);

        // C1: locked on the source clip -> winner must stay in it.
        {
            MotionMatchingResult result;
            String error;
            FindBest(database, queryPose, queryTrajectory, lockedSettings, sourceClip, 0.0f, noTags, result, error);
            phaseCTest++;
            tested++;
            const bool ok = error.IsEmpty() && result.IsValid && result.ClipIndex == sourceClip;
            if (ok)
            {
                phaseCPass++;
                passed++;
            }
            else
            {
                log += String::Format(
                    TEXT("Continuity FAIL: sample {} clip {} (winner clip={}, err='{}').\n"),
                    source,
                    sourceClip,
                    result.ClipIndex,
                    error);
            }
        }

        // C2: tag include -> winner overlaps the source set (exactly what
        // the overlap filter guarantees; tie twins may carry a subset, so a
        // superset assert would false-alarm on identical-feature ties).
        {
            const Array<String> include = sourceTags;
            MotionMatchingResult result;
            String error;
            FindBest(database, queryPose, queryTrajectory, settings, -1, MAX_float, include, result, error);
            phaseCTest++;
            tested++;
            const Array<String> winnerTags = database->GetClipTags(result.ClipIndex);
            bool overlap = false;
            for (const String& tag : include)
            {
                if (winnerTags.Contains(tag))
                {
                    overlap = true;
                    break;
                }
            }
            const bool ok = error.IsEmpty() && result.IsValid && overlap;
            if (ok)
            {
                phaseCPass++;
                passed++;
            }
            else
            {
                log += String::Format(
                    TEXT("Tag-include FAIL: sample {} tag '{}' (winner clip={}, err='{}').\n"),
                    source,
                    sourceTag,
                    result.ClipIndex,
                    error);
            }
        }

        // C3: tag exclude -> winner must carry none of the source tags.
        // Uses the disjoint ExcludeTags settings (not the overlap include
        // scope: with multi-tags a winner can pass an overlap filter via one
        // tag while carrying a forbidden one).
        {
            MotionMatchingSearchSettings exclSettings = settings;
            exclSettings.ExcludeTags = sourceTags;
            MotionMatchingResult result;
            String error;
            FindBest(database, queryPose, queryTrajectory, exclSettings, -1, MAX_float, noTags, result, error);
            phaseCTest++;
            tested++;
            const Array<String> winnerTags = database->GetClipTags(result.ClipIndex);
            bool disjoint = true;
            for (const String& tag : sourceTags)
            {
                if (winnerTags.Contains(tag))
                {
                    disjoint = false;
                    break;
                }
            }
            const bool ok = error.IsEmpty() && result.IsValid && disjoint;
            if (ok)
            {
                phaseCPass++;
                passed++;
            }
            else
            {
                log += String::Format(
                    TEXT("Tag-exclude FAIL: sample {} tag '{}' (winner clip={}, err='{}').\n"),
                    source,
                    sourceTag,
                    result.ClipIndex,
                    error);
            }
        }
    }
    log += String::Format(TEXT("Phase C (continuity/tags): {}/{} passed.\n"), phaseCPass, phaseCTest);

    // Phase D: temporal exclusion + per-block telemetry. Querying the exact
    // source sample while standing on it (CurrentTime == its time) must NOT
    // return a nearby frame on the same clip � otherwise playback reseeks
    // every query instead of playing through. The winner must be outside the
    // window or on another clip, and Pose+Trajectory+Continuity must sum to Cost.
    int32 phaseDTest = 0;
    int32 phaseDPass = 0;
    {
        MotionMatchingSearchSettings exclSettings = settings;
        exclSettings.CurrentTime = 0.0f; // overwritten per probe
        exclSettings.SameClipExclusionWindow = 0.25f;
        const Array<float>& sampleTimesArr = database->GetSampleTimes();
        int32 probes = 0;
        for (int32 source = 0; source < sampleCount && probes < 12; source += 4999)
        {
            const int32 sourceClip = sampleClipIndices[source];
            buildExactQuery(source, queryPose, queryTrajectory);
            exclSettings.CurrentTime = sampleTimesArr[source];
            MotionMatchingResult result;
            String error;
            FindBest(database, queryPose, queryTrajectory, exclSettings, sourceClip, MAX_float, noTags, result, error);
            phaseDTest++;
            tested++;
            probes++;
            bool ok = error.IsEmpty() && result.IsValid;
            if (ok)
            {
                // Same-clip winners must be outside the exclusion window.
                if (result.ClipIndex == sourceClip)
                {
                    const float dt = result.SampleTime - sampleTimesArr[source];
                    if (dt < 0.0f ? -dt <= 0.250001f : dt <= 0.250001f)
                        ok = false;
                }
                // Per-block telemetry must reconstruct the total and stay ordered
                // (LoopCost included: rest preference is a real block now).
                const float blockSum = result.PoseCost + result.TrajectoryCost + result.ContinuityCost + result.LoopCost;
                if (!std::isfinite(result.PoseCost) || !std::isfinite(result.TrajectoryCost) ||
                    !std::isfinite(result.ContinuityCost) || !std::isfinite(result.LoopCost) ||
                    std::fabs(blockSum - result.Cost) > 0.001f ||
                    result.SecondBestCost < result.Cost)
                    ok = false;
            }
            if (ok)
            {
                phaseDPass++;
                passed++;
            }
            else
            {
                log += String::Format(
                    TEXT("Exclusion/telemetry FAIL: sample {} clip {} time {:.3f} (winner clip={} time={:.3f} cost={:.4f} pose={:.4f} traj={:.4f} cont={:.4f} second={:.4f} err='{}').\n"),
                    source,
                    sourceClip,
                    sampleTimesArr[source],
                    result.ClipIndex,
                    result.SampleTime,
                    result.Cost,
                    result.PoseCost,
                    result.TrajectoryCost,
                    result.ContinuityCost,
                    result.SecondBestCost,
                    error);
            }
        }
        // Exclusion disabled (window 0) must recover the exact source sample
        // with ~zero cost � proves exclusion is the only steering here.
        for (int32 source = 0; source < sampleCount && probes < 16; source += 19999)
        {
            const int32 sourceClip = sampleClipIndices[source];
            buildExactQuery(source, queryPose, queryTrajectory);
            MotionMatchingSearchSettings noExcl = settings;
            noExcl.CurrentTime = sampleTimesArr[source];
            noExcl.SameClipExclusionWindow = 0.0f;
            MotionMatchingResult result;
            String error;
            FindBest(database, queryPose, queryTrajectory, noExcl, sourceClip, MAX_float, noTags, result, error);
            phaseDTest++;
            tested++;
            probes++;
            // Same verdict contract as exact queries (ties allowed but ordered).
            bool ok = error.IsEmpty() && result.IsValid && result.Cost <= SelfTestMaxCost;
            if (ok)
            {
                phaseDPass++;
                passed++;
            }
            else
            {
                log += String::Format(
                    TEXT("Exclusion-off FAIL: sample {} (winner={}, cost={:.6f}, err='{}').\n"),
                    source,
                    result.SampleIndex,
                    result.Cost,
                    error);
            }
        }
    }
    log += String::Format(TEXT("Phase D (temporal exclusion/telemetry): {}/{} passed.\n"), phaseDPass, phaseDTest);

    // Phase E: live trajectory frame parity. A constant actor->root offset
    // (mesh mount, stripped-root placement) must not bias the query: after
    // BuildRawTrajectory the zero-time point is exactly (0,0)/(0,1).
    int32 phaseETest = 0;
    int32 phaseEPass = 0;
    {
        const PoseSearchSchema& bakedSchema = database->GetSchema();
        int32 zeroIdx = -1;
        for (int32 i = 0; i < bakedSchema.TrajectorySampleTimes.Count(); i++)
        {
            if (std::fabs(bakedSchema.TrajectorySampleTimes[i]) <= 0.0001f)
            {
                zeroIdx = i;
                break;
            }
        }
        // Synthetic actor-local walk line with a large constant offset,
        // simulating a mesh mount 100 units right / 50 forward of the actor.
        TrajectoryPoint synth[PoseSearchSchema::TrajectoryPointCount];
        for (int32 i = 0; i < PoseSearchSchema::TrajectoryPointCount; i++)
        {
            synth[i].Position = Float3(100.0f + i * 20.0f, 0.0f, 50.0f + i * 30.0f);
            synth[i].Facing = Float3(0.0f, 0.0f, 1.0f);
        }
        Matrix actorWorld = Matrix::Identity;
        Matrix rootWorld = Matrix::Identity;
        rootWorld.SetTranslation(Float3(100.0f, 0.0f, 50.0f)); // root sits at the offset origin
        float raw[PoseSearchSchema::GetTrajectoryFeatureSize()];
        String trajError;
        phaseETest++;
        tested++;
        if (MotionMatchingQuery::BuildRawTrajectory(synth, actorWorld, rootWorld, bakedSchema, raw, trajError))
        {
            const int32 zi = (zeroIdx >= 0 ? zeroIdx : PoseSearchSchema::TrajectoryPointCount / 2) *
                PoseSearchSchema::TrajectoryFloatsPerPoint;
            const bool zeroOk = std::fabs(raw[zi]) <= 0.0001f &&
                std::fabs(raw[zi + 1]) <= 0.0001f &&
                std::fabs(raw[zi + 2]) <= 0.0001f &&
                std::fabs(raw[zi + 3] - 1.0f) <= 0.0001f;
            bool allFinite = true;
            for (int32 i = 0; i < PoseSearchSchema::GetTrajectoryFeatureSize(); i++)
                allFinite &= std::isfinite(raw[i]);
            if (zeroOk && allFinite)
            {
                phaseEPass++;
                passed++;
            }
            else
            {
                log += String::Format(
                    TEXT("Trajectory-rebase FAIL: zero=({:.4f},{:.4f},{:.4f},{:.4f}) finite={}.\n"),
                    raw[zi], raw[zi + 1], raw[zi + 2], raw[zi + 3], allFinite ? 1 : 0);
            }
        }
        else
        {
            log += String::Format(TEXT("Trajectory-rebase FAIL: builder error '{}'.\n"), trajError);
        }
    }
    log += String::Format(TEXT("Phase E (trajectory zero-time rebase): {}/{} passed.\n"), phaseEPass, phaseETest);

    // Phase F: top-2 correctness vs a no-prune reference. The hot loop prunes
    // at secondBestCost; recompute the two lowest FullSampleCost values without
    // pruning or continuity and require exact agreement on best and runner-up.
    int32 phaseFTest = 0;
    int32 phaseFPass = 0;
    {
        int32 probes = 0;
        for (int32 source = 123; source < sampleCount && probes < 5; source += 19999)
        {
            buildExactQuery(source, queryPose, queryTrajectory);
            MotionMatchingResult result;
            String error;
            FindBest(database, queryPose, queryTrajectory, settings, -1, MAX_float, noTags, result, error);
            phaseFTest++;
            tested++;
            probes++;
            bool ok = error.IsEmpty() && result.IsValid;
            if (ok)
            {
                float refBest = MAX_float;
                float refSecond = MAX_float;
                for (int32 s = 0; s < sampleCount; s++)
                {
                    const float c = FullSampleCost(*database, queryPose, queryTrajectory, settings, -1, s, true);
                    if (c < refBest)
                    {
                        refSecond = refBest;
                        refBest = c;
                    }
                    else if (c < refSecond)
                    {
                        refSecond = c;
                    }
                }
                if (std::fabs(result.Cost - refBest) > 0.0005f ||
                    std::fabs(result.SecondBestCost - refSecond) > 0.0005f)
                    ok = false;
            }
            if (ok)
            {
                phaseFPass++;
                passed++;
            }
            else
            {
                log += String::Format(
                    TEXT("Top2 FAIL: sample {} (cost={:.6f} second={:.6f}, err='{}').\n"),
                    source,
                    result.Cost,
                    result.SecondBestCost,
                    error);
            }
        }
    }
    log += String::Format(TEXT("Phase F (top-2 reference): {}/{} passed.\n"), phaseFPass, phaseFTest);

    // Phase G: continuation bias, switch margin, exclusion exemption.
    // Winner assertions use clip + cost (not exact index): static clips hold
    // bit-identical twins, so any in-family winner with the biased cost
    // proves the mechanism; exact-index checks would false-alarm on ties.
    int32 phaseGTest = 0;
    int32 phaseGPass = 0;
    {
        const Array<float>& sampleTimesArr = database->GetSampleTimes();
        const Array<int32>& clipSampleCounts = database->GetClipSampleCounts();
        const int32 clipCount = database->GetClipCount();
        Array<int32> clipFirst;
        clipFirst.Resize(clipCount);
        int32 cursor = 0;
        for (int32 c = 0; c < clipCount; c++)
        {
            clipFirst[c] = cursor;
            cursor += clipSampleCounts[c];
        }
        // Structural probes (NOT absolute sample offsets): middle samples of
        // spread low-turn clips. Absolute offsets rot with every rebake
        // (clip order follows tag grouping), landing probes in turn clips
        // where TurnPenalty legitimately breaks the -0.99 assertions. Each
        // probe is self-consistency pre-checked (exact query returns its own
        // clip), so bit-identical twins can never false-alarm the family.
        struct GProbe { int32 sample; int32 clip; };
        Array<GProbe> gProbes;
        MotionMatchingSearchSettings gPre = settings;
        gPre.CurrentTime = -1.0f;
        gPre.ExpectedTime = -1.0f;
        gPre.SameClipExclusionWindow = 0.0f;
        gPre.ContinuingBias = 0.0f;
        gPre.ContinuingWindow = 0.2f;
        gPre.SwitchMargin = 0.0f;
        const int32 gStep = clipCount / 12 > 0 ? clipCount / 12 : 1;
        for (int32 c = 0; c < clipCount && gProbes.Count() < 6; c += gStep)
        {
            const int32 count = c < clipSampleCounts.Count() ? clipSampleCounts[c] : 0;
            if (count < 40)
                continue;
            if (Math::Abs(database->GetClipTurnAngle(c)) >= 0.05f)
                continue;
            const int32 mid = clipFirst[c] + count / 2;
            buildExactQuery(mid, queryPose, queryTrajectory);
            MotionMatchingResult gPreResult;
            String gPreError;
            FindBest(database, queryPose, queryTrajectory, gPre, -1, 999.0f, noTags, gPreResult, gPreError);
            if (gPreError.IsEmpty() && gPreResult.IsValid && gPreResult.ClipIndex == c &&
                gPreResult.Cost <= SelfTestMaxCost)
            {
                GProbe p;
                p.sample = mid;
                p.clip = c;
                gProbes.Add(p);
            }
        }
        for (int32 pi = 0; pi < gProbes.Count(); pi++)
        {
            const int32 source = gProbes[pi].sample;
            const int32 C = gProbes[pi].clip;
            const float t_s = sampleTimesArr[source];
            const int32 D = gProbes[(pi + 1) % gProbes.Count()].clip;
            const float t_d = sampleTimesArr[clipFirst[D]];
            buildExactQuery(source, queryPose, queryTrajectory);

            // G1: bias rewards the natural continuation (cost � -bias).
            {
                MotionMatchingSearchSettings g = settings;
                g.CurrentTime = t_s - 0.1f;
                g.ExpectedTime = t_s;
                g.SameClipExclusionWindow = 0.0f;
                g.ContinuingBias = 1.0f;
                g.ContinuingWindow = 0.2f;
                g.SwitchMargin = 0.0f;
                MotionMatchingResult result;
                String error;
                FindBest(database, queryPose, queryTrajectory, g, C, 999.0f, noTags, result, error);
                phaseGTest++;
                tested++;
                const float blockSum = result.PoseCost + result.TrajectoryCost + result.ContinuityCost + result.LoopCost;
                bool ok = error.IsEmpty() && result.IsValid && result.ClipIndex == C &&
                    result.Cost < -0.99f && result.ContinuityCost < -0.99f &&
                    std::fabs(blockSum - result.Cost) <= 0.001f;
                if (ok)
                {
                    phaseGPass++;
                    passed++;
                }
                else
                {
                    log += String::Format(
                        TEXT("Bias FAIL: sample {} (winner={} clip={} cost={:.4f} cont={:.4f} err='{}').\n"),
                        source,
                        result.SampleIndex,
                        result.ClipIndex,
                        result.Cost,
                        result.ContinuityCost,
                        error);
                }
            }
            // G2: huge margin demotes the exact winner to a continuation;
            // the demoted exact sample must become runner-up � 0.
            // Continuity is zeroed: this probes margin mechanics only, and a
            // +0.25 other-clip penalty on the exact sample would pollute the
            // runner-up assertion below.
            {
                MotionMatchingSearchSettings g = settings;
                g.CurrentTime = -1.0f;
                g.ExpectedTime = t_d;
                g.SameClipExclusionWindow = 0.0f;
                g.ContinuingBias = 0.0f;
                g.ContinuityWeight = 0.0f;
                g.ContinuingWindow = 0.2f;
                g.SwitchMargin = 1000000.0f;
                MotionMatchingResult result;
                String error;
                FindBest(database, queryPose, queryTrajectory, g, D, 999.0f, noTags, result, error);
                phaseGTest++;
                tested++;
                bool ok = error.IsEmpty() && result.IsValid && result.ClipIndex == D &&
                    result.SecondBestCost <= SelfTestMaxCost;
                if (ok)
                {
                    phaseGPass++;
                    passed++;
                }
                else
                {
                    log += String::Format(
                        TEXT("Margin FAIL: sample {} (winner={} clip={} second={:.6f} err='{}').\n"),
                        source,
                        result.SampleIndex,
                        result.ClipIndex,
                        result.SecondBestCost,
                        error);
                }
            }
            // G2 control: zero margin keeps the exact family winning. Pure
            // cost (no current clip at all) so continuity cannot steer it.
            {
                MotionMatchingSearchSettings g = settings;
                g.CurrentTime = -1.0f;
                g.ExpectedTime = -1.0f;
                g.SameClipExclusionWindow = 0.0f;
                g.ContinuingBias = 0.0f;
                g.ContinuingWindow = 0.2f;
                g.SwitchMargin = 0.0f;
                MotionMatchingResult result;
                String error;
                FindBest(database, queryPose, queryTrajectory, g, -1, 999.0f, noTags, result, error);
                phaseGTest++;
                tested++;
                bool ok = error.IsEmpty() && result.IsValid && result.ClipIndex == C &&
                    result.Cost <= SelfTestMaxCost;
                if (ok)
                {
                    phaseGPass++;
                    passed++;
                }
                else
                {
                    log += String::Format(
                        TEXT("Margin-control FAIL: sample {} (winner={} clip={} err='{}').\n"),
                        source,
                        result.SampleIndex,
                        result.ClipIndex,
                        error);
                }
            }
            // G3: continuation is exempt from temporal exclusion.
            {
                MotionMatchingSearchSettings g = settings;
                g.CurrentTime = t_s;
                g.ExpectedTime = t_s;
                g.SameClipExclusionWindow = 0.25f;
                g.ContinuingBias = 1.0f;
                g.ContinuingWindow = 0.2f;
                g.SwitchMargin = 0.0f;
                MotionMatchingResult result;
                String error;
                FindBest(database, queryPose, queryTrajectory, g, C, 999.0f, noTags, result, error);
                phaseGTest++;
                tested++;
                bool ok = error.IsEmpty() && result.IsValid && result.ClipIndex == C &&
                    result.Cost < -0.99f;
                if (ok)
                {
                    phaseGPass++;
                    passed++;
                }
                else
                {
                    log += String::Format(
                        TEXT("Exempt FAIL: sample {} (winner={} clip={} cost={:.4f} err='{}').\n"),
                        source,
                        result.SampleIndex,
                        result.ClipIndex,
                        result.Cost,
                        error);
                }
            }
        }
    }
    log += String::Format(TEXT("Phase G (continuation/margin): {}/{} passed.\n"), phaseGPass, phaseGTest);

    // Phase H: rest-loop preference. An exact query of a NON-loop end stance
    // (break/stop tail) with a huge preference must resolve to a loop clip;
    // with zero preference the exact one-shot wins. Proves the break-loop
    // lock has a structural exit, not just better-tuned weights.
    int32 phaseHTest = 0;
    int32 phaseHPass = 0;
    {
        // Collect one non-loop end sample per tag that has loop clips.
        struct EndProbe { int32 sample; int32 clip; };
        Array<EndProbe> probes;
        const Array<int32>& clipSampleCounts = database->GetClipSampleCounts();
        const int32 clipCount = database->GetClipCount();
        int32 cursor = 0;
        for (int32 c = 0; c < clipCount && probes.Count() < 8; c++)
        {
            const int32 count = clipSampleCounts[c];
            if (count > 10 && !database->GetClipLoop(c))
            {
                EndProbe p;
                p.sample = cursor + count - 1;
                p.clip = c;
                probes.Add(p);
            }
            cursor += count;
        }
        for (int32 i = 0; i < probes.Count(); i++)
        {
            const int32 source = probes[i].sample;
            buildExactQuery(source, queryPose, queryTrajectory);
            // H1: huge preference -> winner must be a loop clip.
            {
                MotionMatchingSearchSettings g = settings;
                g.CurrentTime = -1.0f;
                g.ExpectedTime = -1.0f;
                g.SameClipExclusionWindow = 0.0f;
                g.ContinuingBias = 0.0f;
                g.SwitchMargin = 0.0f;
                g.LoopPreference = 1000000.0f;
                MotionMatchingResult result;
                String error;
                FindBest(database, queryPose, queryTrajectory, g, -1, 999.0f, noTags, result, error);
                phaseHTest++;
                tested++;
                bool ok = error.IsEmpty() && result.IsValid &&
                    database->GetClipLoop(result.ClipIndex) &&
                    result.LoopCost <= -999999.0f;
                if (ok)
                {
                    phaseHPass++;
                    passed++;
                }
                else
                {
                    log += String::Format(
                        TEXT("LoopPref FAIL: sample {} clip {} (winner={} clip={} loop={} loopCost={:.1f} err='{}').\n"),
                        source,
                        probes[i].clip,
                        result.SampleIndex,
                        result.ClipIndex,
                        database->GetClipLoop(result.ClipIndex) ? 1 : 0,
                        result.LoopCost,
                        error);
                }
            }
            // H2 control: zero preference -> exact one-shot wins.
            {
                MotionMatchingSearchSettings g = settings;
                g.CurrentTime = -1.0f;
                g.ExpectedTime = -1.0f;
                g.SameClipExclusionWindow = 0.0f;
                g.ContinuingBias = 0.0f;
                g.SwitchMargin = 0.0f;
                g.LoopPreference = 0.0f;
                MotionMatchingResult result;
                String error;
                FindBest(database, queryPose, queryTrajectory, g, -1, 999.0f, noTags, result, error);
                phaseHTest++;
                tested++;
                bool ok = error.IsEmpty() && result.IsValid && result.ClipIndex == probes[i].clip &&
                    result.Cost <= SelfTestMaxCost && result.LoopCost == 0.0f;
                if (ok)
                {
                    phaseHPass++;
                    passed++;
                }
                else
                {
                    log += String::Format(
                        TEXT("LoopPref-control FAIL: sample {} clip {} (winner={} clip={} cost={:.6f} err='{}').\n"),
                        source,
                        probes[i].clip,
                        result.SampleIndex,
                        result.ClipIndex,
                        result.Cost,
                        error);
                }
            }
            // H3: end-recovery shape (LoopOnly, zero preference, idle tags):
            // exact one-shot end queries must resolve to a loop clip even
            // with NO bonus helping � the filter alone exits the lock.
            {
                MotionMatchingSearchSettings g = settings;
                g.CurrentTime = -1.0f;
                g.ExpectedTime = -1.0f;
                g.SameClipExclusionWindow = 0.0f;
                g.ContinuingBias = 0.0f;
                g.SwitchMargin = 0.0f;
                g.LoopPreference = 0.0f;
                                g.CollectClassDiagnostics = true;
                                g.LoopFilter = MotionLoopFilter::LoopOnly;
                Array<String> idleOnly;
                idleOnly.Add(String(TEXT("idle")));
                MotionMatchingResult result;
                String error;
                FindBest(database, queryPose, queryTrajectory, g, -1, 999.0f, idleOnly, result, error);
                phaseHTest++;
                tested++;
                bool ok = error.IsEmpty() && result.IsValid &&
                    database->GetClipLoop(result.ClipIndex) &&
                    database->GetClipTags(result.ClipIndex).Contains(String(TEXT("idle")));
                if (ok)
                {
                    phaseHPass++;
                    passed++;
                }
                else
                {
                    log += String::Format(
                        TEXT("EndRecover FAIL: sample {} clip {} (winner={} clip={} tag='{}' err='{}').\n"),
                        source,
                        probes[i].clip,
                        result.SampleIndex,
                        result.ClipIndex,
                        MotionMatchingDatabase::JoinTags(database->GetClipTags(result.ClipIndex)),
                        error);
                }
            }
        }
    }
    log += String::Format(TEXT("Phase H (loop preference): {}/{} passed.\n"), phaseHPass, phaseHTest);

    // Phase I: loop filter + class telemetry. Exact queries keep the true
    // class minima near zero so prune can never hide them (pruned members
    // are invisible to class telemetry by design � see SearchPass).
    int32 phaseITest = 0;
    int32 phaseIPass = 0;
    {
        // Deterministic probe picks (spread probing would rarely land on the
        // ~6% loop samples): middle samples of known loop / non-loop clips.
        const Array<int32>& clipSampleCounts = database->GetClipSampleCounts();
        const int32 clipCount = database->GetClipCount();
        Array<int32> loopProbes;
        Array<int32> oneProbes;
        int32 cursor = 0;
        for (int32 c = 0; c < clipCount && (loopProbes.Count() < 3 || oneProbes.Count() < 3); c++)
        {
            const int32 count = clipSampleCounts[c];
            if (count > 10)
            {
                if (database->GetClipLoop(c) && loopProbes.Count() < 3)
                    loopProbes.Add(cursor + count / 2);
                else if (!database->GetClipLoop(c) && oneProbes.Count() < 3)
                    oneProbes.Add(cursor + count / 2);
            }
            cursor += count;
        }
        // I1: LoopOnly on exact one-shot queries -> loop winners, one-shot
        // telemetry untouched, loop class scored.
        for (int32 i = 0; i < oneProbes.Count(); i++)
        {
            const int32 source = oneProbes[i];
            buildExactQuery(source, queryPose, queryTrajectory);
            MotionMatchingSearchSettings g = settings;
            g.CurrentTime = -1.0f;
            g.ExpectedTime = -1.0f;
            g.SameClipExclusionWindow = 0.0f;
            g.LoopPreference = 0.0f;
            g.CollectClassDiagnostics = true;
            g.LoopFilter = MotionLoopFilter::LoopOnly;
            MotionMatchingResult result;
            String error;
            FindBest(database, queryPose, queryTrajectory, g, -1, 999.0f, noTags, result, error);
            phaseITest++;
            tested++;
            bool ok = error.IsEmpty() && result.IsValid &&
                database->GetClipLoop(result.ClipIndex) &&
                result.BestOneShotSample == -1 && result.OneShotScoredCount == 0 &&
                result.LoopScoredCount > 0;
            if (ok)
            {
                phaseIPass++;
                passed++;
            }
            else
            {
                log += String::Format(
                    TEXT("LoopOnly FAIL: sample {} (winner={} loop={} oneshotN={} loopN={} err='{}').\n"),
                    source,
                    result.SampleIndex,
                    database->GetClipLoop(result.ClipIndex) ? 1 : 0,
                    result.OneShotScoredCount,
                    result.LoopScoredCount,
                    error);
            }
        }
        // I2: OneShotOnly on exact loop queries -> non-loop winners.
        for (int32 i = 0; i < loopProbes.Count(); i++)
        {
            const int32 source = loopProbes[i];
            buildExactQuery(source, queryPose, queryTrajectory);
            MotionMatchingSearchSettings g = settings;
            g.CurrentTime = -1.0f;
            g.ExpectedTime = -1.0f;
            g.SameClipExclusionWindow = 0.0f;
            g.LoopPreference = 0.0f;
            g.CollectClassDiagnostics = true;
            g.LoopFilter = MotionLoopFilter::OneShotOnly;
            MotionMatchingResult result;
            String error;
            FindBest(database, queryPose, queryTrajectory, g, -1, 999.0f, noTags, result, error);
            phaseITest++;
            tested++;
            bool ok = error.IsEmpty() && result.IsValid &&
                !database->GetClipLoop(result.ClipIndex) &&
                result.BestLoopSample == -1 && result.LoopScoredCount == 0 &&
                result.OneShotScoredCount > 0;
            if (ok)
            {
                phaseIPass++;
                passed++;
            }
            else
            {
                log += String::Format(
                    TEXT("OneShotOnly FAIL: sample {} (winner={} loop={} loopN={} oneshotN={} err='{}').\n"),
                    source,
                    result.SampleIndex,
                    database->GetClipLoop(result.ClipIndex) ? 1 : 0,
                    result.LoopScoredCount,
                    result.OneShotScoredCount,
                    error);
            }
        }
        // I3: class telemetry on plain exact queries. The tight near-zero
        // bound (which implies a scored member) applies ONLY to the source's
        // own class � its global-min twin always completes. The opposite
        // class may be legitimately fully pruned (nothing near an exact
        // query survives a collapsed runner-up), so it only needs finite
        // values; I4 below covers both-classes-scored with a blurry query.
        for (int32 round = 0; round < 2; round++)
        {
            const Array<int32>& set = round == 0 ? loopProbes : oneProbes;
            for (int32 i = 0; i < set.Count(); i++)
            {
                const int32 source = set[i];
                const bool sourceLoop = round == 0;
                buildExactQuery(source, queryPose, queryTrajectory);
                MotionMatchingSearchSettings g = settings;
                g.CurrentTime = -1.0f;
                g.ExpectedTime = -1.0f;
                g.SameClipExclusionWindow = 0.0f;
                g.LoopPreference = 0.0f;
                                g.CollectClassDiagnostics = true;
                                g.LoopFilter = MotionLoopFilter::Any;
                MotionMatchingResult result;
                String error;
                FindBest(database, queryPose, queryTrajectory, g, -1, 999.0f, noTags, result, error);
                phaseITest++;
                tested++;
                bool ok = error.IsEmpty() && result.IsValid &&
                    std::isfinite(result.BestLoopCost) && std::isfinite(result.BestOneShotCost) &&
                    (sourceLoop
                        ? (result.BestLoopCost <= SelfTestMaxCost + 0.0005f &&
                           result.BestLoopClip >= 0 && result.LoopScoredCount > 0)
                        : (result.BestOneShotCost <= SelfTestMaxCost + 0.0005f &&
                           result.BestOneShotClip >= 0 && result.OneShotScoredCount > 0));
                if (ok)
                {
                    phaseIPass++;
                    passed++;
                }
                else
                {
                    log += String::Format(
                        TEXT("ClassTel FAIL: sample {} (loop={:.4f} one={:.4f} loopN={} oneN={} err='{}').\n"),
                        source,
                        result.BestLoopCost,
                        result.BestOneShotCost,
                        result.LoopScoredCount,
                        result.OneShotScoredCount,
                        error);
                }
            }
        }
        // I4 dropped (was: blurry midpoint query must land in a parent
        // family). Unsound premise: in 42-D feature space the midpoint of two
        // distant samples is routinely nearest to a THIRD family (observed:
        // midpoint of loop+oneshot won by an unrelated clip with oneN=0).
        // That is legitimate search behavior, not a telemetry failure �
        // both-classes-scored is query-dependent by design (see prune
        // honesty note). I1�I3 already cover filter mechanics deterministically.
    }
    log += String::Format(TEXT("Phase I (loop filter/telemetry): {}/{} passed.\n"), phaseIPass, phaseITest);

    // Phase J: full-scan class minima vs an independent FullSampleCost
    // reference (raw blocks, same tag+filter scope). Proves BestLoopCost /
    // BestOneShotCost report exact minima, not pruned leftovers, and that
    // totals match an independent member count.
    int32 phaseJTest = 0;
    int32 phaseJPass = 0;
    {
        const int32 clipCount = database->GetClipCount();
        int32 probes = 0;
        for (int32 source = 9000; source < sampleCount && probes < 4; source += 14999)
        {
            buildExactQuery(source, queryPose, queryTrajectory);
            MotionMatchingSearchSettings g = settings;
            g.CurrentTime = -1.0f;
            g.ExpectedTime = -1.0f;
            g.SameClipExclusionWindow = 0.0f;
            g.LoopPreference = 0.0f;
            g.CollectClassDiagnostics = true;
            g.LoopFilter = MotionLoopFilter::Any;
            MotionMatchingResult result;
            String error;
            FindBest(database, queryPose, queryTrajectory, g, -1, 999.0f, noTags, result, error);
            phaseJTest++;
            tested++;
                bool ok = error.IsEmpty() && result.IsValid;
                float refLoop = MAX_float;
                float refOne = MAX_float;
                int32 refLoopN = 0;
                int32 refOneN = 0;
                if (ok)
                {
                    for (int32 s = 0; s < sampleCount; s++)
                    {
                        const int32 c = sampleClipIndices[s];
                        if (c < 0 || c >= clipCount)
                            continue;
                        const float v = FullSampleCost(*database, queryPose, queryTrajectory, g, -1, s, false);
                        if (database->GetClipLoop(c))
                        {
                            refLoopN++;
                            if (v < refLoop)
                                refLoop = v;
                        }
                        else
                        {
                            refOneN++;
                            if (v < refOne)
                                refOne = v;
                        }
                    }
                    if (std::fabs(result.BestLoopCost - refLoop) > 0.0005f ||
                        std::fabs(result.BestOneShotCost - refOne) > 0.0005f ||
                        result.LoopTotalCount != refLoopN ||
                        result.OneShotTotalCount != refOneN)
                        ok = false;
                }
            if (ok)
            {
                phaseJPass++;
                passed++;
            }
            else
            {
                log += String::Format(
                    TEXT("FullScan FAIL: sample {} (loop={:.4f} vs {:.4f} n={}/{} one={:.4f} vs {:.4f} n={}/{} err='{}').\n"),
                    source,
                    result.BestLoopCost,
                    refLoop,
                    result.LoopTotalCount,
                    refLoopN,
                    result.BestOneShotCost,
                    refOne,
                    result.OneShotTotalCount,
                    refOneN,
                    error);
            }
            probes++;
        }
    }
    log += String::Format(TEXT("Phase J (full-scan reference): {}/{} passed.\n"), phaseJPass, phaseJTest);

    // Phase K: prior term parity. The brute-force reference now uses the SAME
    // FullSampleCost all-terms function as the hot pass, so it covers
    // continuity/bias, loop preference and the foot-lock prior � the cases the
    // old raw-block reference could not see (PLAN �7.2/�18.7).
    int32 phaseKTest = 0;
    int32 phaseKPass = 0;
    {
        struct KCase
        {
            const Char* Label;
            float Continuity;
            float LoopPreference;
            float FootLock;
            int32 CurrentClip;
            float ExpectedTime;
        };
        const Array<float>& sampleTimesArr = database->GetSampleTimes();
        int32 probes = 0;
        for (int32 source = 2000; source < sampleCount && probes < 3; source += 17000)
        {
            const int32 sourceClip = sampleClipIndices[source];
            buildExactQuery(source, queryPose, queryTrajectory);
            const KCase cases[3] =
            {
                { TEXT("continuity"), 1.0f, 0.0f, 0.0f, sourceClip, sampleTimesArr[source] },
                { TEXT("loop"), 0.0f, 100.0f, 0.0f, -1, -1.0f },
                { TEXT("footlock"), 0.0f, 0.0f, 1.0f, -1, -1.0f },
            };
            for (const KCase& kc : cases)
            {
                MotionMatchingSearchSettings k = settings;
                k.ContinuityWeight = kc.Continuity;
                k.CurrentTime = -1.0f;
                k.SameClipExclusionWindow = 0.0f;
                k.ExpectedTime = kc.ExpectedTime;
                k.ContinuingBias = kc.Continuity > 0.0f ? 1.0f : 0.0f;
                k.ContinuingWindow = 0.2f;
                k.SwitchMargin = 0.0f;
                k.TwinBanWindow = 0.0f;
                k.LoopTailExclusion = 0.0f;
                k.LoopPreference = kc.LoopPreference;
                k.FootLockL = kc.FootLock;
                k.FootLockR = kc.FootLock;
                // D1/�5.2: the prior is gated on the firm flag now, so the
                // parity probe must opt in exactly as the runtime does.
                k.FirmLockL = kc.FootLock > 0.0f;
                k.FirmLockR = kc.FootLock > 0.0f;
                MotionMatchingResult result;
                String error;
                FindBest(database, queryPose, queryTrajectory, k, kc.CurrentClip, MAX_float, noTags, result, error);
                phaseKTest++;
                tested++;
                bool ok = error.IsEmpty() && result.IsValid;
                float refBest = MAX_float;
                int32 refBestSample = -1;
                if (ok)
                {
                    for (int32 s = 0; s < sampleCount; s++)
                    {
                        const float c = FullSampleCost(
                            *database, queryPose, queryTrajectory, k, kc.CurrentClip, s, true);
                        if (c < refBest)
                        {
                            refBest = c;
                            refBestSample = s;
                        }
                    }
                    if (std::fabs(result.Cost - refBest) > 0.0005f ||
                        result.SampleIndex != refBestSample)
                        ok = false;
                }
                if (ok)
                {
                    phaseKPass++;
                    passed++;
                }
                else
                {
                    log += String::Format(
                        TEXT("Term-parity({}) FAIL: sample {} (winner={} cost={:.6f} vs ref {:.6f}@{} err='{}').\n"),
                        String(kc.Label),
                        source,
                        result.SampleIndex,
                        result.Cost,
                        refBest,
                        refBestSample,
                        error);
                }
            }
            probes++;
        }
    }
    log += String::Format(TEXT("Phase K (prior term parity): {}/{} passed.\n"), phaseKPass, phaseKTest);

    // Phase L: structural non-loop tail validity. A non-loop sample whose
    // future trajectory horizon reaches past the clip end must never win when
    // ExcludeNonLoopTail is on; the winner must itself have a valid future.
    // (The flag-off exact-tail behavior is Phase A/B's contract.)
    int32 phaseLTest = 0;
    int32 phaseLPass = 0;
    {
        const float horizon = GetTrajectoryHorizon(*database);
        const Array<float>& sampleTimesArr = database->GetSampleTimes();
        const Array<float>& clipLengths = database->GetClipLengths();
        MotionMatchingSearchSettings l = settings;
        l.ExcludeNonLoopTail = true;
        l.CurrentTime = -1.0f;
        l.ExpectedTime = -1.0f;
        l.SameClipExclusionWindow = 0.0f;
        l.ContinuingBias = 0.0f;
        l.SwitchMargin = 0.0f;
        int32 cursor = 0;
        int32 probes = 0;
        for (int32 c = 0; c < clipCount && probes < 6; c++)
        {
            const int32 n = clipSampleCounts[c];
            const int32 source = cursor + (n > 0 ? n - 1 : 0);
            cursor += n;
            if (n <= 1 || database->GetClipLoop(c) || c >= clipLengths.Count())
                continue;
            if (!MotionMatchingSampler::IsNonLoopFutureOutside(
                    sampleTimesArr[source], horizon, clipLengths[c], false))
                continue;
            buildExactQuery(source, queryPose, queryTrajectory);
            MotionMatchingResult result;
            String error;
            FindBest(database, queryPose, queryTrajectory, l, -1, MAX_float, noTags, result, error);
            phaseLTest++;
            tested++;
            bool ok = error.IsEmpty() && result.IsValid;
            if (ok)
            {
                const int32 wc = sampleClipIndices[result.SampleIndex];
                const bool wLoop = wc >= 0 && wc < clipCount && database->GetClipLoop(wc);
                const float wLen = (wc >= 0 && wc < clipLengths.Count()) ? clipLengths[wc] : 0.0f;
                if (MotionMatchingSampler::IsNonLoopFutureOutside(
                        result.SampleTime, horizon, wLen, wLoop))
                    ok = false;
            }
            if (ok)
            {
                phaseLPass++;
                passed++;
            }
            else
            {
                log += String::Format(
                    TEXT("Tail-validity FAIL: tail sample {} clip {} (winner={} clip={} time={:.3f} err='{}').\n"),
                    source,
                    c,
                    result.SampleIndex,
                    result.ClipIndex,
                    result.SampleTime,
                    error);
            }
            probes++;
        }
    }
    log += String::Format(TEXT("Phase L (non-loop tail validity): {}/{} passed.\n"), phaseLPass, phaseLTest);

    const auto endTime = std::chrono::high_resolution_clock::now();
    const double milliseconds = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    log += String::Format(
        TEXT("Self-test: {}/{} checks passed ({} ties verified), worst exact cost={:.6f} at sample {}, {:.1f} ms total ({:.2f} ms/search over {} samples)."),
        passed,
        tested,
        tieCount,
        worstExactCost,
        worstExactSample,
        milliseconds,
        tested > 0 ? milliseconds / tested : 0.0,
        sampleCount);
    return passed == tested && tested > 0;
}
