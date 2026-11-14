#include "reference_tracker.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool isImageFile(const std::string& path)
{
    const std::size_t separator = path.find_last_of("/\\");
    const std::size_t filename_start =
        separator == std::string::npos ? 0 : separator + 1;
    const std::size_t dot = path.find_last_of('.');
    std::string extension = dot == std::string::npos || dot < filename_start
        ? std::string()
        : path.substr(dot);
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::tolower(character));
                   });
    return extension == ".png" || extension == ".jpg" ||
           extension == ".jpeg" || extension == ".bmp";
}

void drawDetection(cv::Mat& frame, const q1::Detection& detection)
{
    for (std::size_t i = 0; i < detection.corners.size(); ++i) {
        const cv::Point start(cvRound(detection.corners[i].x),
                              cvRound(detection.corners[i].y));
        const cv::Point end(cvRound(detection.corners[(i + 1) % detection.corners.size()].x),
                            cvRound(detection.corners[(i + 1) % detection.corners.size()].y));
        cv::line(frame, start, end, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    const auto top_left = std::min_element(
        detection.corners.begin(), detection.corners.end(),
        [](const cv::Point2f& first, const cv::Point2f& second) {
            return first.y < second.y;
        });
    const cv::Point label_position(cvRound(top_left->x),
                                   std::max(20, cvRound(top_left->y) - 8));
    cv::putText(frame, detection.name, label_position, cv::FONT_HERSHEY_SIMPLEX,
                0.65, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: q1_tracking <dossier_images> <fichier_video>\n"
                  << "Exemple: q1_tracking Data/images Data/videos/2-BMW.mp4\n";
        return 2;
    }

    std::string pattern(argv[1]);
    if (!pattern.empty() && pattern.back() != '/' && pattern.back() != '\\') {
#ifdef _WIN32
        pattern += '\\';
#else
        pattern += '/';
#endif
    }
    pattern += '*';

    std::vector<cv::String> entries;
    cv::glob(pattern, entries, false);

    std::vector<std::string> image_paths;
    for (const cv::String& entry : entries) {
        if (isImageFile(entry)) {
            image_paths.push_back(entry);
        }
    }
    std::sort(image_paths.begin(), image_paths.end());

    if (image_paths.empty()) {
        std::cerr << "Aucune image de reference trouvee dans : "
                  << argv[1] << '\n';
        return 2;
    }

    try {
        q1::ReferenceTracker tracker(image_paths);
        cv::VideoCapture video(argv[2]);
        if (!video.isOpened()) {
            std::cerr << "Impossible d'ouvrir la video : " << argv[2] << '\n';
            return 2;
        }

        double video_fps = video.get(cv::CAP_PROP_FPS);
        if (!std::isfinite(video_fps) || video_fps <= 0.0) {
            video_fps = 30.0;
        }
        const int display_delay_ms = std::max(1, cvRound(1000.0 / video_fps));

        cv::Mat frame;
        while (video.read(frame)) {
            for (const q1::Detection& detection : tracker.update(frame)) {
                drawDetection(frame, detection);
            }

            cv::imshow("Question 1 - Identification et tracking", frame);
            const int key = cv::waitKey(display_delay_ms);
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "Erreur : " << error.what() << '\n';
        return 1;
    }

    return 0;
}
