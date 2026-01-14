#pragma once
#include "utility.hpp"
#include "iostream"

#define minDepth 100
#define maxDepth 5000

class YoloTracker {
    int counter = 0;
    float fps = 0;
    cv::Scalar color = cv::Scalar(255, 255, 255);
    std::chrono::steady_clock::time_point startTime;

    std::shared_ptr<dai::node::Camera> camRgb;
    std::shared_ptr<dai::node::Camera> monoLeft;
    std::shared_ptr<dai::node::Camera> monoRight;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    std::shared_ptr<dai::node::SpatialDetectionNetwork> spatialDetectionNetwork;
    std::shared_ptr<dai::MessageQueue> preview;
    std::shared_ptr<dai::MessageQueue> tracklets;
    std::shared_ptr<std::vector<FruitPos>> fruitsVec = std::make_shared<std::vector<FruitPos>>();
public:
    // Create pipeline
    dai::Pipeline pipeline;
    
    YoloTracker(std::string nnarchive) {


        // Define sources and outputs
        camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::nullopt, 10);
        monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 10);
        monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, 10);
        std::cout << "camera setup \n";

        // Create stereo node
        stereo = pipeline.create<dai::node::StereoDepth>();
        auto leftOutput = monoLeft->requestOutput(std::make_pair(640, 400));
        auto rightOutput = monoRight->requestOutput(std::make_pair(640, 400));
        leftOutput->link(stereo->left);
        rightOutput->link(stereo->right);
        std::cout << "stereo setup \n";

        // Create spatial detection network
        //dai::NNArchive model(nnarchive);
        dai::NNModelDescription model{ "yolov6-nano" };
        spatialDetectionNetwork = pipeline.create<dai::node::SpatialDetectionNetwork>()->build(camRgb, stereo, model);
        spatialDetectionNetwork->setConfidenceThreshold(0.8f);
        spatialDetectionNetwork->input.setBlocking(false);
        spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5f);
        spatialDetectionNetwork->setDepthLowerThreshold(100);
        spatialDetectionNetwork->setDepthUpperThreshold(5000);
        std::cout << "spatial detection network made, minimum distance is: " << minDepth / 1000.0f << "m, max distance is: " << maxDepth / 1000.0f << "m \n";


        // Create object tracker
        auto objectTracker = pipeline.create<dai::node::ObjectTracker>();
        objectTracker->setDetectionLabelsToTrack({ 0,1 });  // track only red and green
        objectTracker->setTrackerType(dai::TrackerType::SHORT_TERM_IMAGELESS);
        objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::SMALLEST_ID);
        std::cout << "object tracker started \n";


        // Create output queues
        preview = objectTracker->passthroughTrackerFrame.createOutputQueue();
        tracklets = objectTracker->out.createOutputQueue();

        // Link nodes
        spatialDetectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);
        spatialDetectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
        spatialDetectionNetwork->out.link(objectTracker->inputDetections);
        std::cout << "links made \n";


        // Start pipeline
        pipeline.start();
        std::cout << "pipeline started \n";
    }

    std::shared_ptr<std::vector<FruitPos>> getFruitsPtr() { return fruitsVec; };

    std::vector<FruitPos> checkFruit() {
        std::vector<FruitPos> fruits = std::vector<FruitPos>();
        counter++;
        //std::cout << "trying to find a fruit, count: " << counter << "\n";
        fruits.clear();
        auto track = tracklets->get<dai::Tracklets>();
        auto trackletsData = track->tracklets;
        for (const auto& t : trackletsData) {
            if (t.status != dai::Tracklet::TrackingStatus::LOST && t.status != dai::Tracklet::TrackingStatus::REMOVED) {
                FruitPos fruit = FruitPos();
                fruit.x = (t.spatialCoordinates.x) / 1000.0;
                fruit.y = (t.spatialCoordinates.y) / 1000.0;
                fruit.z = (t.spatialCoordinates.z) / 1000.0;
                if (t.label == 1) {
                    fruit.ripe;
                }
                else fruit.ripe = false;
                fruits.push_back(fruit);
            }
        }
        return fruits;
    }
};