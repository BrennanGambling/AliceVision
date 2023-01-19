// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "KeyframeSelector.hpp"
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/dataio/FeedProvider.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <iostream>
#include <random>
#include <tuple>
#include <cassert>
#include <cstdlib>
#include <iomanip>

#include <opencv2/optflow.hpp>
#include <opencv2/imgcodecs.hpp>

namespace fs = boost::filesystem;

namespace aliceVision
{
namespace keyframe
{

/**
 * @brief Get a random int in order to generate uid.
 * @warning The random don't use a repeatable seed to avoid conflicts between different launches on different data sets.
 * @return int between 0 and std::numeric_limits<int>::max()
 */
int getRandomInt()
{
    std::random_device rd;             // will be used to obtain a seed for the random number engine
    std::mt19937 randomTwEngine(rd()); // standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> randomDist(0, std::numeric_limits<int>::max());
    return randomDist(randomTwEngine);
}

double computeSharpness(const cv::Mat& grayscaleImage, const int window_size)
{
    cv::Mat sum, sumsq, laplacian;
    cv::Laplacian(grayscaleImage, laplacian, CV_64F);
    cv::integral(laplacian, sum, sumsq);

    double totalCount = window_size * window_size;
    double maxstd = 0.0;
    for (int y = 0; y < sum.rows - window_size; y++)
    {
        for (int x = 0; x < sum.cols - window_size; x++)
        {
            double tl = sum.at<double>(y, x);
            double tr = sum.at<double>(y, x + window_size);
            double bl = sum.at<double>(y + window_size, x);
            double br = sum.at<double>(y + window_size, x + window_size);
            double s1 = br + tl - tr - bl;

            tl = sumsq.at<double>(y, x);
            tr = sumsq.at<double>(y, x + window_size);
            bl = sumsq.at<double>(y + window_size, x);
            br = sumsq.at<double>(y + window_size, x + window_size);
            double s2 = br + tl - tr - bl;

            double std_2 = std::sqrt((s2 - (s1 * s1) / totalCount) / totalCount);

            maxstd = std::max(maxstd, std_2);
        }
    }

    return maxstd;
}

cv::Mat readImage(dataio::FeedProvider & feed, size_t max_width)
{
    image::Image<image::RGBColor> image;
    camera::PinholeRadialK3 queryIntrinsics;
    bool hasIntrinsics = false;
    std::string currentImgName;

    if (!feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
    {
        ALICEVISION_LOG_ERROR("Cannot read frame '" << currentImgName << "' !");
        throw std::invalid_argument("Cannot read frame '" + currentImgName + "' !");
    }

    // Convert content to OpenCV
    cv::Mat cvFrame(cv::Size(image.cols(), image.rows()), CV_8UC3, image.data(), image.cols() * 3);

    cv::Mat cvGrayscale;
    // Convert to grayscale
    cv::cvtColor(cvFrame, cvGrayscale, cv::COLOR_BGR2GRAY);

    // Resize to smaller size
    cv::Mat cvRescaled;
    if (cvGrayscale.cols > max_width && max_width > 0)
    {
        cv::resize(cvGrayscale, cvRescaled, cv::Size(max_width, double(cvGrayscale.rows) * double(max_width) / double(cvGrayscale.cols)));
    }
    else
    {
        cvRescaled = cvGrayscale;
    }

    return cvRescaled;
}

double findMedian(const std::vector<double>& vec)
{
    std::vector<double> vecCopy = vec;
    if (vecCopy.size() > 0 && vecCopy.size() % 2 == 0)
    {
        const auto medianIt1 = vecCopy.begin() + vecCopy.size() / 2 - 1;
        const auto medianIt2 = vecCopy.begin() + vecCopy.size() / 2;
        std::nth_element(vecCopy.begin(), medianIt1, vecCopy.end());
        const auto med1 = *medianIt1;
        std::nth_element(vecCopy.begin(), medianIt2, vecCopy.end());
        const auto med2 = *medianIt2;
        return (med1 + med2) / 2.0;
    }
    else if (vecCopy.size() > 0)
    {
        const auto medianIt = vecCopy.begin() + vecCopy.size() / 2;
        std::nth_element(vecCopy.begin(), medianIt, vecCopy.end());
        return *medianIt;
    }
    else
    {
        return 0.0;
    }
}

double estimateFlow(const std::vector<std::unique_ptr<dataio::FeedProvider>> & feeds, size_t max_width, int previous, int current)
{
    auto ptrFlow = cv::optflow::createOptFlow_DeepFlow();

    double minmaxflow = std::numeric_limits<double>::max();

    for (auto& feed : feeds)
    {
        if (!feed->goToFrame(previous))
        {
            ALICEVISION_LOG_ERROR("Invalid frame position. Ignoring this frame.");
            return 0.0;
        }

        cv::Mat first = readImage(*feed, max_width);

        if (!feed->goToFrame(current))
        {
            ALICEVISION_LOG_ERROR("Invalid frame position. Ignoring this frame.");
            return 0.0;
        }

        cv::Mat second = readImage(*feed, max_width);

        cv::Mat flow;
        ptrFlow->calc(first, second, flow);

        cv::Mat sumflow;
        cv::integral(flow, sumflow, CV_64F);

        int ws = 20;
        double n = ws * ws;
        double maxflow = 0.0;
        size_t count = 0;

        cv::Mat matnorm(flow.size(), CV_32FC1);
        for (int i = 10; i < flow.rows - ws - 10; i++)
        {
            for (int j = 10; j < flow.cols - ws - 10; j++)
            {
                cv::Point2d tl = sumflow.at<cv::Point2d>(i, j);
                cv::Point2d tr = sumflow.at<cv::Point2d>(i, j + ws);
                cv::Point2d bl = sumflow.at<cv::Point2d>(i + ws, j);
                cv::Point2d br = sumflow.at<cv::Point2d>(i + ws, j + ws);
                cv::Point2d s1 = br + tl - tr - bl;

                cv::Point fl = flow.at<cv::Point2f>(i, j);

                double norm = std::hypot(s1.x, s1.y) / n;
                maxflow = std::max(maxflow, norm);
            }
        }

        minmaxflow = std::min(minmaxflow, maxflow);
    }

    return minmaxflow;
}

void KeyframeSelector::processRegular(const std::vector<std::string>& mediaPaths)
{
    std::size_t nbFrames = std::numeric_limits<std::size_t>::max();
    std::vector<std::unique_ptr<dataio::FeedProvider>> feeds;

    _selected.clear();

    for (std::size_t mediaIndex = 0; mediaIndex < mediaPaths.size(); ++mediaIndex)
    {
        const auto& path = mediaPaths.at(mediaIndex);

        // Create a feed provider per mediaPaths
        feeds.emplace_back(new dataio::FeedProvider(path));

        const auto& feed = *feeds.back();

        // Check if feed is initialized
        if (!feed.isInit())
        {
            ALICEVISION_LOG_ERROR("Cannot initialize the FeedProvider with " << path);
            throw std::invalid_argument("Cannot while initialize the FeedProvider with " + path);
        }

        // Update minimum number of frames
        nbFrames = std::min(nbFrames, (size_t)feed.nbFrames());
    }

    // Check if minimum number of frame is zero
    if (nbFrames == 0)
    {
        ALICEVISION_LOG_ERROR("One or multiple medias can't be found or empty!");
        throw std::invalid_argument("One or multiple medias can't be found or empty!");
    }

    int step = _minFrameStep;
    if (_maxOutFrame > 0 && nbFrames / _maxOutFrame > step)
    {
        step = (nbFrames / _maxOutFrame) + 1;   // + 1 to prevent ending up with more than _maxOutFrame selected frames
    }

    for (int id = 0; id < nbFrames; id += step)
    {
        _selected.push_back(id);
    }
}

void KeyframeSelector::processSmart(const std::vector<std::string> & mediaPaths)
{
    // Create feeds and count minimum number of frames
    std::size_t nbFrames = std::numeric_limits<std::size_t>::max();
    std::vector<std::unique_ptr<dataio::FeedProvider>> feeds;
    const size_t processWidth = 720;
    const double thresholdSharpness = 10.0;
    const double thresholdFlow = 10.0;

    for (std::size_t mediaIndex = 0; mediaIndex < mediaPaths.size(); ++mediaIndex)
    {
        const auto& path = mediaPaths.at(mediaIndex);

        // Create a feed provider per mediaPaths
        feeds.emplace_back(new dataio::FeedProvider(path));

        const auto& feed = *feeds.back();

        // Check if feed is initialized
        if (!feed.isInit())
        {
            ALICEVISION_LOG_ERROR("Cannot initialize the FeedProvider with " << path);
            throw std::invalid_argument("Cannot while initialize the FeedProvider with " + path);
        }

        // Update minimum number of frames
        nbFrames = std::min(nbFrames, (size_t)feed.nbFrames());
    }

    // Check if minimum number of frame is zero
    if (nbFrames == 0)
    {
        ALICEVISION_LOG_ERROR("One or multiple medias can't be found or empty !");
        throw std::invalid_argument("One or multiple medias can't be found or empty !");
    }

    const int searchWindowSize = (_maxOutFrame <= 0) ? 1 : nbFrames / _maxOutFrame;

    // Feed provider variables
    image::Image<image::RGBColor> image;     // original image
    camera::PinholeRadialK3 queryIntrinsics; // image associated camera intrinsics
    bool hasIntrinsics = false;              // true if queryIntrinsics is valid
    std::string currentImgName;              // current image name


    // Feed and metadata initialization
    for (std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex)
    {
        // First frame with offset
        feeds.at(mediaIndex)->goToFrame(0);

        if (!feeds.at(mediaIndex)->readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
        {
            ALICEVISION_LOG_ERROR("Cannot read media first frame " << mediaPaths[mediaIndex]);
            throw std::invalid_argument("Cannot read media first frame " + mediaPaths[mediaIndex]);
        }
    }

    size_t currentFrame = 0;

    std::vector<double> sharpnessScores;

    while (currentFrame < nbFrames)
    {
        double minimalSharpness = std::numeric_limits<double>::max();

        for (std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex)
        {
            ALICEVISION_LOG_DEBUG("media : " << mediaPaths.at(mediaIndex));
            auto& feed = *feeds.at(mediaIndex);

            // Resize to smaller size
            cv::Mat cvRescaled = readImage(feed, processWidth);

            double score = computeSharpness(cvRescaled, 200);
            minimalSharpness = std::min(minimalSharpness, score);

            feed.goToNextFrame();
        }

        sharpnessScores.push_back(minimalSharpness);
        currentFrame++;
    }


    std::vector<unsigned int> indices;

    int startPosition = 0;

    while (startPosition < sharpnessScores.size())
    {
        int endPosition = std::min(int(sharpnessScores.size()), startPosition + searchWindowSize);
        auto maxIter = std::max_element(sharpnessScores.begin() + startPosition, sharpnessScores.begin() + endPosition);
        size_t index = maxIter - sharpnessScores.begin();
        double maxval = *maxIter;
        if (maxval < thresholdSharpness)
        {
            // This value means that the image is completely blurry.
            // We consider we should not select a value here
            startPosition += searchWindowSize;
            continue;
        }

        if (indices.size() == 0)
        {
            // No previous, so no flow check
            startPosition = index + std::max(int(_minFrameStep), searchWindowSize);
            indices.push_back(index);
            continue;
        }

        int previous = indices.back();

        double flow = estimateFlow(feeds, processWidth, previous, index);
        if (flow < thresholdFlow)
        {
            // Continue with next frame
            startPosition = index + 1;
            continue;
        }

        indices.push_back(index);
        startPosition = index + std::max(int(_minFrameStep), searchWindowSize);
    }

    _selected = indices;
}

bool KeyframeSelector::writeSelection(const std::string & outputFolder, const std::vector<std::string>& mediaPaths, const std::vector<std::string>& brands, const std::vector<std::string>& models, const std::vector<float>& mmFocals)
{
    image::Image< image::RGBColor> image;
    camera::PinholeRadialK3 queryIntrinsics;
    bool hasIntrinsics = false;
    std::string currentImgName;

    for (int id = 0; id < mediaPaths.size(); id++)
    {
        const auto& path = mediaPaths.at(id);

        // Create a feed provider per mediaPaths
        dataio::FeedProvider feed(path);

        // Check if feed is initialized
        if (!feed.isInit())
        {
            ALICEVISION_LOG_ERROR("Cannot initialize the FeedProvider with " << path);
            return false;
        }

        std::string processedOutputFolder = outputFolder;
        if (mediaPaths.size() > 1)
        {
            const std::string rigFolder = outputFolder + "/rig/";
            if (!fs::exists(rigFolder))
            {
                fs::create_directory(rigFolder);
            }

            processedOutputFolder = rigFolder + std::to_string(id);
            if (!fs::exists(processedOutputFolder))
            {
                fs::create_directory(processedOutputFolder);
            }
        }

        for (auto pos : _selected)
        {
            if (!feed.goToFrame(pos))
            {   
                ALICEVISION_LOG_ERROR("Invalid frame position. Ignoring this frame.");
                continue;
            }

            if (!feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
            {
                ALICEVISION_LOG_ERROR("Error reading image");
                return false;
            }

            oiio::ImageSpec spec(image.Width(), image.Height(), 3, oiio::TypeDesc::UINT8);

            spec.attribute("Make", brands[id]);
            spec.attribute("Model", models[id]);
            spec.attribute("Exif:BodySerialNumber", std::to_string(getRandomInt())); // TODO: use Exif:OriginalRawFileName instead
            spec.attribute("Exif:FocalLength", mmFocals[id]);
            spec.attribute("Exif:ImageUniqueID", std::to_string(getRandomInt()));

            fs::path folder = outputFolder;
            std::ostringstream filenameSS;
            filenameSS << std::setw(5) << std::setfill('0') << pos << ".exr";
            const auto filepath = (processedOutputFolder / fs::path(filenameSS.str())).string();


            std::unique_ptr<oiio::ImageOutput> out(oiio::ImageOutput::create(filepath));

            if (out.get() == nullptr)
            {
                throw std::invalid_argument("Cannot create image file : " + filepath);
            }

            if (!out->open(filepath, spec))
            {
                throw std::invalid_argument("Cannot open image file : " + filepath);
            }

            out->write_image(oiio::TypeDesc::UINT8, image.data());
            out->close();
        }

    }

    return true;
}

bool KeyframeSelector::exportSharpnessToFile(const std::vector<std::string>& mediaPaths,
                                             const std::string& folder, const std::string& filename) const
{
    // Create feeds and count minimum number of frames
    std::size_t nbFrames = std::numeric_limits<std::size_t>::max();
    std::vector<std::unique_ptr<dataio::FeedProvider>> feeds;

    for (std::size_t mediaIndex = 0; mediaIndex < mediaPaths.size(); ++mediaIndex)
    {
        const auto& path = mediaPaths.at(mediaIndex);

        // Create a feed provider per mediaPaths
        feeds.emplace_back(new dataio::FeedProvider(path));
        const auto& feed = *feeds.back();

        // Check if feed is initialized
        if (!feed.isInit())
        {
            ALICEVISION_LOG_ERROR("Cannot initialize the FeedProvider with " << path);
            throw std::invalid_argument("Cannot while initialize the FeedProvider with " + path);
        }

        // Update minimum number of frames
        nbFrames = std::min(nbFrames, (size_t)feed.nbFrames());
    }

    // Check if minimum number of frame is zero
    if (nbFrames == 0)
    {
        ALICEVISION_LOG_ERROR("One or multiple medias can't be found or empty !");
        throw std::invalid_argument("One or multiple medias can't be found or empty !");
    }

    // Feed provider variables
    image::Image<image::RGBColor> image;     // original image
    camera::PinholeRadialK3 queryIntrinsics; // image associated camera intrinsics
    bool hasIntrinsics = false;              // true if queryIntrinsics is valid
    std::string currentImgName;              // current image name


    // Feed and metadata initialization
    for (std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex)
    {
        // First frame with offset
        feeds.at(mediaIndex)->goToFrame(0);

        if (!feeds.at(mediaIndex)->readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
        {
            ALICEVISION_LOG_ERROR("Cannot read media first frame " << mediaPaths[mediaIndex]);
            throw std::invalid_argument("Cannot read media first frame " + mediaPaths[mediaIndex]);
        }
    }

    size_t currentFrame = 0;
    std::vector<double> sharpnessScores;

    while (currentFrame < nbFrames)
    {
        double minimalSharpness = std::numeric_limits<double>::max();

        for (std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex)
        {
            ALICEVISION_LOG_DEBUG("media : " << mediaPaths.at(mediaIndex));
            auto& feed = *feeds.at(mediaIndex);

            // Read image into an OpenCV Mat
            cv::Mat cvRescaled = readImage(feed, 0);

            double score = computeSharpness(cvRescaled, 200);
            minimalSharpness = std::min(minimalSharpness, score);

            feed.goToNextFrame();
        }

        sharpnessScores.push_back(minimalSharpness);
        currentFrame++;
    }

    // Export the score vector to a CSV file
    std::ofstream os;
    os.open((fs::path(folder) / filename).string(), std::ios::app);

    if (!os.is_open())
    {
        ALICEVISION_LOG_DEBUG("Unable to open the sharpness scores file: '" << filename << "'.");
        return false;
    }

    os.seekp(0, std::ios::end); // Put the cursor at the end
    if (os.tellp() == std::streampos(0)) // 'tellp' returns the cursor's position
    {
        // If the file does not exist, add a header
        os << "FrameNb;SharpnessScore;\n";
    }

    for (size_t s = 0; s < sharpnessScores.size(); s++)
    {
        os << s << ";" << sharpnessScores.at(s) << ";\n";
    }
    os.close();
    return true;
}

bool KeyframeSelector::exportFlowToFile(const std::vector<std::string>& mediaPaths,
                                        const std::string& folder, const std::string& filename) const
{
    // Create feeds and count minimum number of frames
    std::size_t nbFrames = std::numeric_limits<std::size_t>::max();
    std::vector<std::unique_ptr<dataio::FeedProvider>> feeds;

    for (std::size_t mediaIndex = 0; mediaIndex < mediaPaths.size(); ++mediaIndex)
    {
        const auto& path = mediaPaths.at(mediaIndex);

        // Create a feed provider per mediaPaths
        feeds.emplace_back(new dataio::FeedProvider(path));
        const auto& feed = *feeds.back();

        // Check if feed is initialized
        if (!feed.isInit())
        {
            ALICEVISION_LOG_ERROR("Cannot initialize the FeedProvider with " << path);
            throw std::invalid_argument("Cannot while initialize the FeedProvider with " + path);
        }

        // Update minimum number of frames
        nbFrames = std::min(nbFrames, (size_t)feed.nbFrames());
    }

    // Check if minimum number of frame is zero
    if (nbFrames == 0)
    {
        ALICEVISION_LOG_ERROR("One or multiple medias can't be found or empty !");
        throw std::invalid_argument("One or multiple medias can't be found or empty !");
    }

    // Feed provider variables
    image::Image<image::RGBColor> image;     // original image
    camera::PinholeRadialK3 queryIntrinsics; // image associated camera intrinsics
    bool hasIntrinsics = false;              // true if queryIntrinsics is valid
    std::string currentImgName;              // current image name


    // Feed and metadata initialization
    for (std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex)
    {
        // First frame with offset
        feeds.at(mediaIndex)->goToFrame(0);

        if (!feeds.at(mediaIndex)->readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
        {
            ALICEVISION_LOG_ERROR("Cannot read media first frame " << mediaPaths[mediaIndex]);
            throw std::invalid_argument("Cannot read media first frame " + mediaPaths[mediaIndex]);
        }
    }

    size_t currentFrame = 1;
    std::vector<double> flowScores;

    while (currentFrame < nbFrames)
    {
        double flow = estimateFlow(feeds, 0, 0, 1);
        flowScores.push_back(flow);
        currentFrame++;
    }

    // Export the score vector to a CSV file
    std::ofstream os;
    os.open((fs::path(folder) / filename).string(), std::ios::app);

    if (!os.is_open())
    {
        ALICEVISION_LOG_DEBUG("Unable to open the optical flow scores file: '" << filename << "'.");
        return false;
    }

    os.seekp(0, std::ios::end); // Put the cursor at the end
    if (os.tellp() == std::streampos(0)) // 'tellp' returns the cursor's position
    {
        // If the file does not exist, add a header
        os << "FrameNb;OpticalFlowScore;\n";
    }

    for (size_t s = 0; s < flowScores.size(); s++)
    {
        os << s << ";" << flowScores.at(s) << ";\n";
    }
    os.close();
    return true;
}

/*
 * Identical to computeSharpness except that the sliding window moves from half windowSize pixels
 * instead of 1 pixels at each iteration. Results seem identical, but processing is faster.
 */
double computeSharpness2(const cv::Mat& grayscaleImage, const int windowSize)
{
    cv::Mat sum, sumsq, laplacian;
    cv::Laplacian(grayscaleImage, laplacian, CV_64F);
    cv::integral(laplacian, sum, sumsq);

    double totalCount = windowSize * windowSize;
    double maxstd = 0.0;
    for (int y = 0; y < sum.rows - windowSize; y += int(windowSize / 2))
    {
        for (int x = 0; x < sum.cols - windowSize; x += int(windowSize / 2))
        {
            double tl = sum.at<double>(y, x);
            double tr = sum.at<double>(y, x + windowSize);
            double bl = sum.at<double>(y + windowSize, x);
            double br = sum.at<double>(y + windowSize, x + windowSize);
            double s1 = br + tl - tr - bl;

            tl = sumsq.at<double>(y, x);
            tr = sumsq.at<double>(y, x + windowSize);
            bl = sumsq.at<double>(y + windowSize, x);
            br = sumsq.at<double>(y + windowSize, x + windowSize);
            double s2 = br + tl - tr - bl;

            double std_2 = std::sqrt((s2 - (s1 * s1) / totalCount) / totalCount);

            maxstd = std::max(maxstd, std_2);
        }
    }

    return maxstd;
}

double estimateFlow(const cv::Ptr<cv::DenseOpticalFlow>& ptrFlow, const cv::Mat& grayscaleImage,
                    const cv::Mat& previousGrayscaleImage, const int windowSize)
{
    double minmaxflow = std::numeric_limits<double>::max();
    cv::Mat flow;
    ptrFlow->calc(grayscaleImage, previousGrayscaleImage, flow);

    cv::Mat sumflow;
    cv::integral(flow, sumflow, CV_64F);

    double n = windowSize * windowSize;
    double maxflow = 0.0;
    size_t count = 0;

    cv::Mat matnorm(flow.size(), CV_32FC1);
    for (int i = 10; i < flow.rows - windowSize - 10; i++)
    {
        for (int j = 10; j < flow.cols - windowSize - 10; j++)
        {
            cv::Point2d tl = sumflow.at<cv::Point2d>(i, j);
            cv::Point2d tr = sumflow.at<cv::Point2d>(i, j + windowSize);
            cv::Point2d bl = sumflow.at<cv::Point2d>(i + windowSize, j);
            cv::Point2d br = sumflow.at<cv::Point2d>(i + windowSize, j + windowSize);
            cv::Point2d s1 = br + tl - tr - bl;

            cv::Point fl = flow.at<cv::Point2f>(i, j);

            double norm = std::hypot(s1.x, s1.y) / n;
            maxflow = std::max(maxflow, norm);
        }
    }

    minmaxflow = std::min(minmaxflow, maxflow);
    return minmaxflow;
}

double estimateGlobalFlow(const cv::Ptr<cv::DenseOpticalFlow>& ptrFlow, const cv::Mat& grayscaleImage,
                    const cv::Mat& previousGrayscaleImage)
{
    cv::Mat flow;
    ptrFlow->calc(grayscaleImage, previousGrayscaleImage, flow);

    cv::Mat sumflow;
    cv::integral(flow, sumflow, CV_64F);

    cv::Point2d tl = sumflow.at<cv::Point2d>(0, 0);
    cv::Point2d tr = sumflow.at<cv::Point2d>(0, sumflow.size().width - 1);
    cv::Point2d bl = sumflow.at<cv::Point2d>(sumflow.size().height - 1, 0);
    cv::Point2d br = sumflow.at<cv::Point2d>(sumflow.size().height - 1, sumflow.size().width -1);
    cv::Point2d s1 = br + tl - tr - bl;
    double norm = std::hypot(s1.x, s1.y) / (sumflow.size().width * sumflow.size().height);

    return std::max(0.0, norm);
}

std::vector<double> estimateFlowOnBorders(const cv::Ptr<cv::DenseOpticalFlow>& ptrFlow, const cv::Mat& grayscaleImage,
                        const cv::Mat& previousGrayscaleImage, const uint borderSize)
{
    cv::Mat flow;
    ptrFlow->calc(grayscaleImage, previousGrayscaleImage, flow);

    cv::Mat sumflow;
    cv::integral(flow, sumflow, CV_64F);
    std::vector<double> norms; // top, bottom, left, right, global
    double norm;

    cv::Point2d tlUp = sumflow.at<cv::Point2d>(0, 0);
    cv::Point2d trUp = sumflow.at<cv::Point2d>(0, sumflow.size().width - 1);
    cv::Point2d blUp = sumflow.at<cv::Point2d>(borderSize - 1, 0);
    cv::Point2d brUp = sumflow.at<cv::Point2d>(borderSize - 1, sumflow.size().width - 1);
    cv::Point2d sUp = brUp + tlUp - trUp - blUp;
    norm = std::hypot(sUp.x, sUp.y) / (sumflow.size().width * borderSize);
    norms.push_back(norm);

    cv::Point2d tlDown = sumflow.at<cv::Point2d>(sumflow.size().height - borderSize - 1, 0);
    cv::Point2d trDown = sumflow.at<cv::Point2d>(sumflow.size().height - borderSize - 1, sumflow.size().width - 1);
    cv::Point2d blDown = sumflow.at<cv::Point2d>(sumflow.size().height - 1, 0);
    cv::Point2d brDown = sumflow.at<cv::Point2d>(sumflow.size().height - 1, sumflow.size().width - 1);
    cv::Point2d sDown = brDown + tlDown - trDown - blDown;
    norm = std::hypot(sDown.x, sDown.y) / (sumflow.size().width * borderSize);
    norms.push_back(norm);

    // TODO: test with full left and right borders. Here, there is no overlapping between the left/right and
    // top/bottom borders, so we might be missing out on some extra information for the left and right borders.
    cv::Point2d tlLeft = sumflow.at<cv::Point2d>(borderSize, 0);
    cv::Point2d trLeft = sumflow.at<cv::Point2d>(borderSize, borderSize - 1);
    cv::Point2d blLeft = sumflow.at<cv::Point2d>(sumflow.size().height - borderSize - 2, 0);
    cv::Point2d brLeft = sumflow.at<cv::Point2d>(sumflow.size().height - borderSize - 2, borderSize - 1);
    cv::Point2d sLeft = brLeft + tlLeft - trLeft - blLeft;
    norm = std::hypot(sLeft.x, sLeft.y) / ((sumflow.size().height - (borderSize * 2)) * borderSize);
    norms.push_back(norm);

    cv::Point2d tlRight = sumflow.at<cv::Point2d>(borderSize, sumflow.size().width - 1 - borderSize);
    cv::Point2d trRight = sumflow.at<cv::Point2d>(borderSize, sumflow.size().width - 1);
    cv::Point2d blRight = sumflow.at<cv::Point2d>(sumflow.size().height - borderSize - 2, sumflow.size().width - 1 - borderSize);
    cv::Point2d brRight = sumflow.at<cv::Point2d>(sumflow.size().height - borderSize - 2, sumflow.size().width - 1);
    cv::Point2d sRight = brRight + tlRight - trRight - blRight;
    norm = std::hypot(sRight.x, sRight.y) / ((sumflow.size().height - (borderSize * 2)) * borderSize);
    norms.push_back(norm);

    cv::Point2d tl = sumflow.at<cv::Point2d>(0, 0);
    cv::Point2d tr = sumflow.at<cv::Point2d>(0, sumflow.size().width - 1);
    cv::Point2d bl = sumflow.at<cv::Point2d>(sumflow.size().height - 1, 0);
    cv::Point2d br = sumflow.at<cv::Point2d>(sumflow.size().height - 1, sumflow.size().width -1);
    cv::Point2d s1 = br + tl - tr - bl;
    norm = std::hypot(s1.x, s1.y) / (sumflow.size().width * sumflow.size().height);
    norms.push_back(norm);

    return norms;
}

double estimateFlowByCell(const cv::Ptr<cv::DenseOpticalFlow>& ptrFlow, const cv::Mat& grayscaleImage,
                        const cv::Mat& previousGrayscaleImage, const std::string& folder, const uint cellSize = 90)
{
    cv::Mat flow;
    ptrFlow->calc(grayscaleImage, previousGrayscaleImage, flow);

    cv::Mat sumflow;
    cv::integral(flow, sumflow, CV_64F);
    ALICEVISION_LOG_DEBUG("computed flow for the flow by cell estimation, height of the image: " << sumflow.size().height);

    double norm;
    std::vector<double> motionByCell;

    for (std::size_t i = 0; i < sumflow.size().height; i += cellSize)
    {
        std::size_t maxCellSizeHeight = cellSize;
        if (std::min(sumflow.size().height, int(i + cellSize)) == sumflow.size().height)
            maxCellSizeHeight = sumflow.size().height - i;
        for (std::size_t j = 0; j < sumflow.size().width; j += cellSize)
        {
            cv::Point2d tl = sumflow.at<cv::Point2d>(i, j);
            cv::Point2d tr = sumflow.at<cv::Point2d>(i, j + cellSize - 1);
            cv::Point2d bl = sumflow.at<cv::Point2d>(i + maxCellSizeHeight - 1, j);
            cv::Point2d br = sumflow.at<cv::Point2d>(i + maxCellSizeHeight - 1, j + cellSize - 1);
            cv::Point2d s = br + tl - tr - bl;
            norm = std::hypot(s.x, s.y) / (maxCellSizeHeight * cellSize);
            motionByCell.push_back(norm);
        }
    }

    // OF visualisation
    ALICEVISION_LOG_DEBUG("Frame to output: " << folder);
    cv::Mat flowParts[2];
    cv::split(flow, flowParts);
    cv::Mat magnitude, angle, magnNorm;
    cv::cartToPolar(flowParts[0], flowParts[1], magnitude, angle, true);
    cv::normalize(magnitude, magnNorm, 0.0f, 1.0f, cv::NORM_MINMAX);
    angle *= ((1.f / 360.f) * (180.f / 255.f));

    cv::Mat _hsv[3], hsv, hsv8, bgr;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magnNorm;
    cv::merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);
    cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);
    cv::imwrite(folder, bgr);
    return findMedian(motionByCell);
}

bool KeyframeSelector::computeScores(const std::vector<std::string>& mediaPaths, const std::string& outputFolder,
    bool rescale, bool flowOnBorders, bool flowByCell)
{
    // Reset frame size
    frameWidth = 0;
    frameHeight = 0;

    _sharpnessScores.clear();
    _sharpnessScoresRescaled.clear();
    _flowScores.clear();
    _flowScoresRescaled.clear();
    _flowScoresOnTopBorder.clear();
    _flowScoresOnBottomBorder.clear();
    _flowScoresOnLeftBorder.clear();
    _flowScoresOnRightBorder.clear();
    _flowScoresByCell.clear();

    // Create feeds and count minimum number of frames
    std::size_t nbFrames = std::numeric_limits<std::size_t>::max();
    std::vector<std::unique_ptr<dataio::FeedProvider>> feeds;

    for (std::size_t mediaIndex = 0; mediaIndex < mediaPaths.size(); ++mediaIndex)
    {
        const auto& path = mediaPaths.at(mediaIndex);

        // Create a feed provider per mediaPaths
        feeds.emplace_back(new dataio::FeedProvider(path));
        const auto& feed = *feeds.back();

        // Check if feed is initialized
        if (!feed.isInit())
        {
            ALICEVISION_LOG_ERROR("Cannot initialize the FeedProvider with " << path);
            throw std::invalid_argument("Cannot while initialize the FeedProvider with " + path);
        }

        // Update minimum number of frames
        nbFrames = std::min(nbFrames, (size_t)feed.nbFrames());
    }

    // Check if minimum number of frame is zero
    if (nbFrames == 0)
    {
        ALICEVISION_LOG_ERROR("One or multiple medias can't be found or empty !");
        throw std::invalid_argument("One or multiple medias can't be found or empty !");
    }

    // Feed provider variables
    image::Image<image::RGBColor> image;     // original image
    camera::PinholeRadialK3 queryIntrinsics; // image associated camera intrinsics
    bool hasIntrinsics = false;              // true if queryIntrinsics is valid
    std::string currentImgName;              // current image name

    // Feed and metadata initialization
    for (std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex)
    {
        // First frame with offset
        feeds.at(mediaIndex)->goToFrame(0);

        if (!feeds.at(mediaIndex)->readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
        {
            ALICEVISION_LOG_ERROR("Cannot read media first frame " << mediaPaths[mediaIndex]);
            throw std::invalid_argument("Cannot read media first frame " + mediaPaths[mediaIndex]);
        }
    }

    size_t currentFrame = 0;
    cv::Mat previous, cvMat;
    cv::Mat previousRescaled, cvRescaled;
    auto ptrFlow = cv::optflow::createOptFlow_DeepFlow();

    while (currentFrame < nbFrames)
    {
        double minimalSharpness = std::numeric_limits<double>::max();
        double minimalSharpnessRescaled = std::numeric_limits<double>::max();
        double flow = -1.;
        double flowRescaled = -1.;
        double flowRescaledTop = -1.;
        double flowRescaledBottom = -1.;
        double flowRescaledLeft = -1.;
        double flowRescaledRight = -1.;
        double flowByCellScore = -1.;

        for (std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex)
        {
            ALICEVISION_LOG_DEBUG("media : " << mediaPaths.at(mediaIndex));
            auto& feed = *feeds.at(mediaIndex);

            if (currentFrame > 0)
            {
                previous = cvMat;
                if (rescale)
                    previousRescaled = cvRescaled;
            }

            cvMat = readImage(feed, 0); // Read image at full res
            // double sharpness = computeSharpness2(cvMat, int(cvMat.size().width / (720 / 200)));
            // minimalSharpness = std::min(minimalSharpness, sharpness);
            minimalSharpness = -1.0;

            if (rescale)
            {
                cvRescaled = readImage(feed, 720); // Read image and rescale it
                if (frameWidth == 0 && frameHeight == 0)
                {
                    frameWidth = cvRescaled.size().width;
                    frameHeight = cvRescaled.size().height;
                }
                double sharpnessRescaled = computeSharpness2(cvRescaled, 200);
                minimalSharpnessRescaled = std::min(minimalSharpnessRescaled, sharpnessRescaled);
            }

            if (currentFrame > 0)
            {
                // flow = estimateFlow(ptrFlow, cvMat, previous, int(cvMat.size().width / (720 / 20)));
                // flow = estimateGlobalFlow(ptrFlow, cvMat, previous);
                flow = -1.0;
                if (rescale)
                    // flowRescaled = estimateFlow(ptrFlow, cvRescaled, previousRescaled, 20);
                    if (flowOnBorders) {
                        std::vector<double> flowBorders = estimateFlowOnBorders(ptrFlow, cvRescaled, previousRescaled, 100);
                        flowRescaled = flowBorders.back();
                        flowBorders.pop_back();
                        flowRescaledRight = flowBorders.back();
                        flowBorders.pop_back();
                        flowRescaledLeft = flowBorders.back();
                        flowBorders.pop_back();
                        flowRescaledBottom = flowBorders.back();
                        flowBorders.pop_back();
                        flowRescaledTop = flowBorders.back();
                    } else {
                        flowRescaled = estimateGlobalFlow(ptrFlow, cvRescaled, previousRescaled);
                    }

                    if (flowByCell) {
                        flowByCellScore = estimateFlowByCell(ptrFlow, cvRescaled, previousRescaled, outputFolder + "/" + std::to_string((int)currentFrame) + ".png");
                    }
            }

            feed.goToNextFrame();
        }

        _sharpnessScores.push_back(minimalSharpness);
        _flowScores.push_back(flow);
        if (rescale)
        {
            _sharpnessScoresRescaled.push_back(minimalSharpnessRescaled);
            _flowScoresRescaled.push_back(flowRescaled);
            if (flowOnBorders)
            {
                _flowScoresOnTopBorder.push_back(flowRescaledTop);
                _flowScoresOnBottomBorder.push_back(flowRescaledBottom);
                _flowScoresOnLeftBorder.push_back(flowRescaledLeft);
                _flowScoresOnRightBorder.push_back(flowRescaledRight);
            }

            if (flowByCell)
                _flowScoresByCell.push_back(flowByCellScore);
        }
        currentFrame++;
        ALICEVISION_LOG_DEBUG("Finished processing frame " << currentFrame << "/" << nbFrames);
    }
    return true;
}

void KeyframeSelector::selectFrames(bool refine)
{
    _selected.clear();
    std::vector<unsigned int> unrefinedSelection = selectFramesWithMotion();
    std::vector<unsigned int> refinedSelection;

    if (refine)
        refinedSelection = refineFrameSelection(unrefinedSelection);
    else
        refinedSelection = unrefinedSelection;

    _selected = refinedSelection;
}

void KeyframeSelector::selectFrames(float pxDisplacement, unsigned int minFrames, unsigned int maxFrames)
{
    /**
     * - Step 1: split the whole sequence into subsequences depending on the accumulated movement and the number of wanted frames
     * - Step 2: for each subsequence, find the sharpest frame (combination of weights on the center of the subsequence + sharpness)
     */

    _selected.clear();

    // STEP 1
    // Split sequence depending on accumulated motion first
    std::vector<unsigned int> subsetLimits;
    subsetLimits.push_back(0);

    std::size_t sequenceSize = _flowScoresByCell.size();

    unsigned int step = static_cast<uint>(pxDisplacement * (std::min(frameWidth, frameHeight)) / 100);
    double acc = 0.0;
    for (std::size_t i = 1; i < sequenceSize; i++)
    {
        acc += _flowScoresByCell.at(i);
        if (acc >= step)
        {
            subsetLimits.push_back(i);
            acc = 0.0; // reset accumulator
        }
    }
    subsetLimits.push_back(sequenceSize - 1);

    ALICEVISION_LOG_DEBUG("Number of subsequences: " << subsetLimits.size());

    // Then check whether min/maxFrames constraints are respected
    if (!(subsetLimits.size() - 1 >= minFrames && subsetLimits.size() - 1 <= maxFrames))
    {
        std::vector<unsigned int> newLimits;
        // fix subsequences
        if (subsetLimits.size() - 1 < minFrames)
        {
            // Not enough frames, sample regularly the whole sequence to get minFrames subsequences
            // TODO: improve (reduce motion step instead of sampling regularly?)
            newLimits.push_back(0);
            std::size_t stepSize = (sequenceSize / minFrames) + 1;
            for (std::size_t i = 1; i < sequenceSize; i += stepSize)
            {
                newLimits.push_back(i);
            }
            newLimits.push_back(sequenceSize - 1);
        }
        else
        {
            // Too many frames, increase the motion step between each subsequence
            while (newLimits.size() - 1 > maxFrames)
            {
                newLimits.clear();
                newLimits.push_back(0);
                step++;  // The displacement must be 1px bigger than the one used previously
                acc = 0.0;
                for (std::size_t i = 1; i < sequenceSize; i++)
                {
                    acc += _flowScoresByCell.at(i);
                    if (acc >= step)
                    {
                        newLimits.push_back(i);
                        acc = 0.0;  // reset accumulator
                    }
                }
                newLimits.push_back(sequenceSize - 1);
            }
        }

        subsetLimits.clear();
        subsetLimits = newLimits;
    }

    // STEP 2
    for (std::size_t i = 1; i < subsetLimits.size(); i++)
    {
        ALICEVISION_LOG_DEBUG("Interval [" << subsetLimits.at(i - 1) << ", " << subsetLimits.at(i) << "]");
        double bestSharpness = 0.0;
        std::size_t bestIndex = 0;
        std::size_t subsequenceSize = subsetLimits.at(i) - subsetLimits.at(i - 1);

        // Weights for the whole sequence, between 1 and 2
        std::deque<double> weights;
        double weightStep = 1.f / (static_cast<double>(subsequenceSize - 1) / 2.f);
        weights.push_back(2.0);
        if (subsequenceSize % 2 == 0)
            weights.push_back(2.0);

        float currentWeight = 2.0;
        while (weights.size() != subsetLimits.at(i) - subsetLimits.at(i - 1))
        {
            currentWeight -= weightStep;
            weights.push_front(currentWeight);
            weights.push_back(currentWeight);
        }

        std::size_t weightPosition = 0;
        for (std::size_t j = subsetLimits.at(i - 1); j < subsetLimits.at(i); j++)
        {
            auto sharpness = _sharpnessScoresRescaled.at(j) * weights.at(weightPosition);
            weightPosition++;
            if (sharpness > bestSharpness)
            {
                bestIndex = j;
                bestSharpness = sharpness;
            }
        }
        _selected.push_back(bestIndex);
    }
}

std::vector<unsigned int> KeyframeSelector::selectFramesWithMotion()
{
    std::vector<unsigned int> selectedFrames;
    double scoreTop, scoreBottom, scoreLeft, scoreRight;
    ALICEVISION_LOG_DEBUG("Start selecting frames with motion");

    // TODO: imperfect medians, as they include the -1 value for the first frame
    double medianTop = findMedian(_flowScoresOnTopBorder);
    double medianBottom = findMedian(_flowScoresOnBottomBorder);
    double medianLeft = findMedian(_flowScoresOnLeftBorder);
    double medianRight = findMedian(_flowScoresOnRightBorder);

    ALICEVISION_LOG_DEBUG("Median scores (top, bottom, left, right): " << medianTop << " " << medianBottom << " " <<
        medianLeft << " " << medianRight);

    for (std::size_t i = 0; i < _flowScoresOnTopBorder.size(); i++)
    {
        scoreTop = _flowScoresOnTopBorder.at(i);
        scoreBottom = _flowScoresOnBottomBorder.at(i);
        scoreLeft = _flowScoresOnLeftBorder.at(i);
        scoreRight = _flowScoresOnRightBorder.at(i);

        if (scoreTop >= medianTop && scoreBottom >= medianBottom &&
            scoreLeft >= medianLeft && scoreRight >= medianRight)
        {
            ALICEVISION_LOG_DEBUG("Selecting frame " << i << "(" << scoreTop << " " << scoreBottom << " " <<
                scoreLeft  << " " << scoreRight << ")");
            selectedFrames.push_back(i);
        }
        else
        {
            ALICEVISION_LOG_DEBUG("Rejecting frame " << i << "(" << scoreTop << " " << scoreBottom << " " <<
                scoreLeft  << " " << scoreRight << ")");
        }
    }

    ALICEVISION_LOG_DEBUG("Finished selecting frames with motion");
    return selectedFrames;
}

std::vector<std::vector<unsigned int>> clusterize(const std::vector<unsigned int>& vec, unsigned int limit)
{
    std::vector<std::vector<unsigned int>> clusters;
    if (vec.empty())
        return clusters;

    for (auto it = begin(vec);;)
    {
        auto const last = std::adjacent_find(it, end(vec),
            [limit](unsigned int a, unsigned int b) { return b - a > limit; });

        if (last == end(vec))
        {
            clusters.emplace_back(it, last);
            return clusters;
        }

        auto const gap = next(last);
        clusters.emplace_back(it, gap);
        it = gap;
    }

    // Should never escape the "for" loop without returning but just in case:
    return clusters;
}

std::vector<unsigned int> KeyframeSelector::refineFrameSelection(const std::vector<unsigned int>& selectedFrames)
{
    std::vector<unsigned int> refinedSelection;

    if (selectedFrames.size() > _internalMaxFrames)
    {
        ALICEVISION_LOG_DEBUG("Initial selection contains too many frames");
    }

    std::vector<std::vector<unsigned int>> clusters = clusterize(selectedFrames, _internalFrameClusterLimit);
    ALICEVISION_LOG_DEBUG("Finished clusterizing, about to list final clusters (" << clusters.size() << " clusters found)");
    for (std::size_t i = 0; i < clusters.size(); i++)
    {
        ALICEVISION_LOG_DEBUG("Cluster #" << i + 1 << "/" << clusters.size() << ": {" << clusters.at(i).front() << " "
            << clusters.at(i).back() << "} " << clusters.at(i).size() << " frames");
    }

    // For each cluster, try to find the sharpest frames while respecting _internalMinFrameSteps
    for (std::size_t i = 0; i < clusters.size(); i++)
    {
        double sharpnessScore;
        double maxSharpnessScore = 0.0;
        unsigned int sharpestFrame = 0;

        // TODO: select more than 1 frame per cluster
        std::size_t maxFrames = (clusters.at(i).size() / _internalMinFrameStep) + 1;
        ALICEVISION_LOG_DEBUG("Max frames for cluster " << i + 1 << "/" << clusters.size() << ": " << maxFrames);

        // for (std::size_t j = 0; j < clusters.at(i).size(); j++)
        // {
        //     unsigned int frameId = clusters.at(i).at(j);
        //     sharpnessScore = _sharpnessScoresRescaled.at(frameId);
        //     if (sharpnessScore > maxSharpnessScore)
        //     {
        //         maxSharpnessScore = sharpnessScore;
        //         sharpestFrame = frameId;
        //     }
        // }
        // refinedSelection.push_back(sharpestFrame);

        // Split every cluster in maxFrames parts, and find the sharpest frame in each subsection
        std::vector<double> bestSharpnessScores(maxFrames, 0.0);
        std::vector<unsigned int> bestFrames(maxFrames, 0);

        // TODO: improve performance
        if (maxFrames == 1)
        {
            for (std::size_t j = 0; j < clusters.at(i).size(); j++)
            {
                unsigned int frameId = clusters.at(i).at(j);
                ALICEVISION_LOG_DEBUG("Evaluating sharpness for frame " << frameId << " (cluster " << i + 1 << ")");
                sharpnessScore = _sharpnessScoresRescaled.at(frameId);
                if (sharpnessScore > maxSharpnessScore)
                {
                    maxSharpnessScore = sharpnessScore;
                    sharpestFrame = frameId;
                }
            }
            refinedSelection.push_back(sharpestFrame);
        }
        else
        {
            for (std::size_t j = 0; j < clusters.at(i).size(); j += _internalMinFrameStep)
            {
                for (std::size_t k = j;
                    k < (j + _internalMinFrameStep >= clusters.at(i).size() - 1)
                    ? clusters.at(i).size() : j + _internalMinFrameStep;
                    k++)
                {
                    unsigned int frameId = clusters.at(i).at(k);
                    ALICEVISION_LOG_DEBUG("Evaluating sharpness for frame " << frameId << " (cluster " << i + 1 << ")");
                    sharpnessScore = _sharpnessScoresRescaled.at(frameId);
                    if (sharpnessScore > maxSharpnessScore)
                    {
                        maxSharpnessScore = sharpnessScore;
                        sharpestFrame = frameId;
                    }
                }
                refinedSelection.push_back(sharpestFrame);
            }
        }
    }

    return refinedSelection;
}

bool KeyframeSelector::exportAllScoresToFile(const std::string& folder, bool flowOnBorders) const
{
    std::vector<std::vector<double>> scores;
    scores.push_back(_sharpnessScores);
    scores.push_back(_sharpnessScoresRescaled);
    scores.push_back(_flowScores);
    scores.push_back(_flowScoresRescaled);

    std::string header;
    std::vector<double> selectedFrames(_sharpnessScores.size(), 0);
    for (std::size_t i = 0; i < _selected.size(); i++)
    {
        selectedFrames.at(_selected.at(i)) = 1;
    }

    if (flowOnBorders)
    {
        scores.push_back(_flowScoresOnTopBorder);
        scores.push_back(_flowScoresOnBottomBorder);
        scores.push_back(_flowScoresOnLeftBorder);
        scores.push_back(_flowScoresOnRightBorder);
        header = "FrameNb;Sharpness;SharpnessRescaled;OpticalFlow;OpticalFlowRescaled;OFRescaledTop;OFRescaledBottom;OFRescaledLeft;OFRescaledRight;SelectedFrame;\n";
    } else
    {
        header = "FrameNb;Sharpness;SharpnessRescaled;OpticalFlow;OpticalFlowRescaled;SelectedFrame;\n";
    }

    scores.push_back(selectedFrames);

    return exportScoresToFile(scores, folder, "scores.csv", header);
}

bool KeyframeSelector::exportFlowByCellScores(const std::string &folder) const
{
    std::vector<std::vector<double>> scores;
    scores.push_back(_sharpnessScoresRescaled);
    scores.push_back(_flowScoresRescaled);
    scores.push_back(_flowScoresByCell);

    std::string header = "FrameNb;SharpnessRescaled;OpticalFlowRescaled;OpticalFlowByCell;\n";
    return exportScoresToFile(scores, folder, "scores.csv", header);
}

bool KeyframeSelector::exportScoresToFile(const std::vector<std::vector<double>>& scores,
                                          const std::string& folder, const std::string& filename,
                                          const std::string& header) const
{
    // Export the score vector to a CSV file
    std::ofstream os;
    os.open((fs::path(folder) / filename).string(), std::ios::app);

    if (!os.is_open())
    {
        ALICEVISION_LOG_DEBUG("Unable to open the optical flow scores file: '" << filename << "'.");
        return false;
    }

    os.seekp(0, std::ios::end); // Put the cursor at the end
    if (os.tellp() == std::streampos(0)) // 'tellp' returns the cursor's position
    {
        // If the file does not exist, add a header
        os << header;
    }

    for (size_t s = 0; s < scores.at(0).size(); s++)
    {
        os << s << ";";
        for (size_t ss = 0; ss < scores.size(); ss++)
        {
            os << scores.at(ss).at(s) << ";";
        }
        os << "\n";
    }
    os.close();
    return true;
}

} // namespace keyframe
} // namespace aliceVision
