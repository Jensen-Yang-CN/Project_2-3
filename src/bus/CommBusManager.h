#pragma once

#include <memory>

#include "BerthDetectionNode.h"
#include "BridgeDetectionNode.h"
#include "PreProcessorNode_v2.h"
#include "SensorPublisherNode.h"

/** 统一管理 Topic 总线、预处理节点、桥洞/泊位检测节点 */
class CommBusManager {
public:
    static CommBusManager &instance();

    void ensurePublisher();
    void ensureInitialized();
    void ensureBridgeNode();
    void ensureBerthNode();
    void startPipeline();
    void stopPipeline();
    void requestPlaybackTimelineReset(uint64_t sensorTimelineGeneration = 0);
    bool pipelineRunning() const { return pipeline_running_; }

    SensorPublisherNode *sensorPublisher();
    BridgeDetectionNode *bridgeNode() { return bridge_node_.get(); }
    BerthDetectionNode *berthNode() { return berth_node_.get(); }

private:
    CommBusManager() = default;

    bool core_inited_ = false;
    bool pipeline_running_ = false;

    std::unique_ptr<SensorPublisherNode> publisher_;
    std::unique_ptr<usv::PreProcessorNodeV2> preprocessor_;
    std::unique_ptr<BridgeDetectionNode> bridge_node_;
    std::unique_ptr<BerthDetectionNode> berth_node_;
};
