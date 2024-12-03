// vision.cpp
#include "finalproject/vision.hpp"

Vision::Vision(const std::string &node_name)
    : Node(node_name),
      yellow_line_detected(false), // 노란선 감지
      white_line_detected(false),  // 흰 선 감지
      yellow_line_count(0),        // 노란선 카운트
      white_line_count(0),         // 흰선 카운트
      array_index(0),              // 배열 숫자(라인을 실제로 감지했는지, 배열에 저장해서 계속 유지되는 값인지 확인)
      yellow_line_valid(false),    // 배열 검사에 따른 노란선 유효값
      white_line_valid(false),     // 배열 검사에 따른 흰선 유효값
      yellow_line_x(0.0f),         // 노란선의 x좌표
      white_line_x(0.0f)           // 흰선의 x좌표 두 좌표를 통해 선의 각도 산출
{
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw", 10,
        std::bind(&Vision::imageCallback, this, std::placeholders::_1));

    original_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/original", 10);
    yellow_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/yellow_mask", 10);
    white_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/white_mask", 10);
    line_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/line_detection", 10);

    yellow_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision/yellow_line_detected", 10);
    white_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision/white_line_detected", 10);

    yellow_pos_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vision/yellow_line_x", 10);
    white_pos_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vision/white_line_x", 10);

    blue_sign_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision/blue_sign_detected", 10);

    white_line_points_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/vision/white_line_points", 10);
    yellow_line_points_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/vision/yellow_line_points", 10);

    yellow_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vision/yellow_line_angle", 10);
    white_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vision/white_line_angle", 10);

    barrier_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision/barrier_detected", 10);

    barrier_yellow_line_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision/barrier_yellow_line_detected", 10);
    barrier_white_line_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision/barrier_white_line_detected", 10);
    barrier_yellow_line_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vision/barrier_yellow_line_angle", 10);
    barrier_white_line_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vision/barrier_white_line_angle", 10);

    yellow_detection_array.fill(false);
    white_detection_array.fill(false);
    // 각종 publisher들 master에게 보내주는 부분.
}

bool Vision::isLineValid(std::array<bool, 10> &detection_array, bool current_detection)
{
    // 현재 감지 결과를 배열에 저장
    detection_array[array_index] = current_detection; // 감지 결과 저장 배열

    // true의 개수 계산
    int detection_count = 0;
    for (bool detection : detection_array)
    {
        if (detection)
        {
            detection_count++;
        }
    }

    // 특정값 이상이면 유효한 선으로 판단
    return detection_count >= DETECTION_THRESHOLD;
}
void Vision::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) // 본격적인 이미지에 파라미터 설정 부분
{
    try
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat resized_frame;
        cv::resize(frame, resized_frame, cv::Size(640, 480));

        int height = resized_frame.rows;
        int width = resized_frame.cols;

        // ROI 설정
        cv::Point2f src_vertices[4];
        cv::Point2f dst_vertices[4];
        cv::Point2f signs_vertices[4];
        cv::Point2f bar_vertices[4];

        // 차단바
        bar_vertices[0] = cv::Point2f(width * 0.35f, height * 0.55f);
        bar_vertices[1] = cv::Point2f(width * 0.65f, height * 0.55f);
        bar_vertices[2] = cv::Point2f(width * 0.65f, height * 0.7f);
        bar_vertices[3] = cv::Point2f(width * 0.35f, height * 0.7f);

        // 표지판
        signs_vertices[0] = cv::Point2f(width * 0.95f, height * 0.3f);
        signs_vertices[1] = cv::Point2f(width * 1.0f, height * 0.3f);
        signs_vertices[2] = cv::Point2f(width * 1.0f, height * 0.55f);
        signs_vertices[3] = cv::Point2f(width * 0.95f, height * 0.55f);

        // 라인트레이싱
        src_vertices[0] = cv::Point2f(width * 0.1f, height * 0.9f);
        src_vertices[1] = cv::Point2f(width * 0.9f, height * 0.9f);
        src_vertices[2] = cv::Point2f(width * 0.95f, height * 1.0f);
        src_vertices[3] = cv::Point2f(width * 0.05f, height * 1.0f);
        // 이미지 자체의 크기
        dst_vertices[0] = cv::Point2f(0, 0);
        dst_vertices[1] = cv::Point2f(width, 0);
        dst_vertices[2] = cv::Point2f(width, height);
        dst_vertices[3] = cv::Point2f(0, height);

        // 버드아이뷰 변환
        cv::Mat perspective_matrix = cv::getPerspectiveTransform(src_vertices, dst_vertices);
        cv::Mat birds_eye_view;
        // 버드 아이뷰 각도를 실제 각도로 보정해서 보게 해줌.
        cv::warpPerspective(resized_frame, birds_eye_view, perspective_matrix, cv::Size(width, height));

        // 전처리 과정 (블러 2번)
        cv::Mat preprocessed;
        cv::GaussianBlur(birds_eye_view, preprocessed, cv::Size(5, 5), 0);
        cv::GaussianBlur(birds_eye_view, preprocessed, cv::Size(5, 5), 0);

        // CLAHE 적용 (L*a*b* 색공간) //노이즈 제거 but 색조 대비는 별로 없어서 큰 영향 X
        cv::Mat lab;
        cv::cvtColor(preprocessed, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> lab_channels;
        cv::split(lab, lab_channels);

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.5, cv::Size(8, 8));
        clahe->apply(lab_channels[0], lab_channels[0]);

        // 통합
        cv::merge(lab_channels, lab);
        cv::cvtColor(lab, preprocessed, cv::COLOR_Lab2BGR);

        // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        cv::Mat hsv;
        cv::cvtColor(preprocessed, hsv, cv::COLOR_BGR2HSV);

        // 노란색 HSV,Lab, RGB 3가지로 색을 받아서 혼합.
        cv::Mat yellow_mask_combined;

        cv::Mat yellow_mask_hsv;
        cv::Scalar lower_yellow_hsv(20, 150, 150);
        cv::Scalar upper_yellow_hsv(30, 255, 255);
        cv::inRange(hsv, lower_yellow_hsv, upper_yellow_hsv, yellow_mask_hsv);

        cv::Mat yellow_mask_lab;
        cv::inRange(lab, cv::Scalar(150, 130, 140), cv::Scalar(250, 140, 200), yellow_mask_lab);

        cv::Mat yellow_mask_rgb;
        cv::inRange(preprocessed, cv::Scalar(180, 180, 0), cv::Scalar(255, 255, 150), yellow_mask_rgb);

        cv::bitwise_or(yellow_mask_hsv, yellow_mask_lab, yellow_mask_combined);
        cv::bitwise_or(yellow_mask_combined, yellow_mask_rgb, yellow_mask_combined);

        // 모폴로지 연산, 커널 사이즈 지정
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat kernel_large = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

        cv::morphologyEx(yellow_mask_combined, yellow_mask_combined, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(yellow_mask_combined, yellow_mask_combined, cv::MORPH_CLOSE, kernel_large);

        // 흰색 HSV, Lab, RGB 3가지로 색을 받아서 혼합.
        cv::Mat white_mask_combined;

        // 1. HSV 기반 흰색 검출
        cv::Mat white_mask_hsv;
        cv::Scalar lower_white_hsv(0, 0, 200);
        cv::Scalar upper_white_hsv(180, 30, 255);
        cv::inRange(hsv, lower_white_hsv, upper_white_hsv, white_mask_hsv);

        // 2. Lab 기반 흰색 검출
        cv::Mat white_mask_lab;
        std::vector<cv::Mat> lab_channels_white;
        cv::split(lab, lab_channels_white);
        cv::threshold(lab_channels_white[0], white_mask_lab, 235, 255, cv::THRESH_BINARY);

        // 3. RGB 기반 흰색 검출
        cv::Mat white_mask_rgb;
        cv::inRange(preprocessed, cv::Scalar(240, 240, 240), cv::Scalar(255, 255, 255), white_mask_rgb);

        // 노란색 마스크 제외, dilate로 노이즈 조금 제거
        cv::Mat yellow_mask_dilated;
        cv::dilate(yellow_mask_combined, yellow_mask_dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
        // or 연산
        cv::bitwise_or(white_mask_hsv, white_mask_lab, white_mask_combined);
        cv::bitwise_or(white_mask_combined, white_mask_rgb, white_mask_combined);
        cv::bitwise_and(white_mask_combined, ~yellow_mask_dilated, white_mask_combined);

        cv::morphologyEx(white_mask_combined, white_mask_combined, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(white_mask_combined, white_mask_combined, cv::MORPH_CLOSE, kernel_large);
        cv::dilate(white_mask_combined, white_mask_combined, kernel, cv::Point(-1, -1), 2);

        // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        //  선 검출 (컨투어 기반)
        std::vector<std::vector<cv::Point>> yellow_contours, white_contours;
        cv::findContours(yellow_mask_combined, yellow_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(white_mask_combined, white_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 컨투어 필터링 및 선 그리기
        cv::Mat line_display = preprocessed.clone();

        // 기본 설정
        yellow_line_detected = false;
        white_line_detected = false;
        yellow_line_count = 0;
        white_line_count = 0;

        // 노란색 선 그리기
        for (const auto &contour : yellow_contours)
        {
            double area = cv::contourArea(contour);
            if (area > 200.0) // 작은 노이즈 제거
            {
                yellow_line_detected = true;
                yellow_line_count++;

                // 이미지 픽셀 강도 평균
                cv::Moments moments = cv::moments(contour);
                if (moments.m00 != 0)
                {
                    yellow_line_x = moments.m10 / moments.m00;
                    yellow_line_x = (yellow_line_x / width) * 2 - 1;

                    auto yellow_pos_msg = std_msgs::msg::Float32();
                    yellow_pos_msg.data = yellow_line_x;
                    yellow_pos_pub_->publish(yellow_pos_msg);
                }

                std::vector<cv::Point> approx;
                cv::approxPolyDP(contour, approx, 10, true); // 형상을 근사해서 좀 더 매끄럽게

                // 긴 축을 찾기 위한 최소 영역 사각형
                cv::RotatedRect rot_rect = cv::minAreaRect(contour);
                cv::Point2f vertices[4];
                rot_rect.points(vertices);

                // 좌표 보내주는 부분
                auto line_points_msg = std_msgs::msg::Float32MultiArray();
                line_points_msg.data.resize(8);
                for (int i = 0; i < 4; i++)
                {
                    float x = vertices[i].x;
                    float y = vertices[i].y;

                    line_points_msg.data[i * 2] = x;
                    line_points_msg.data[i * 2 + 1] = y;
                }

                float angle = 0.0f;
                float dx = vertices[2].x - vertices[1].x;
                float dy = vertices[2].y - vertices[1].y;
                // 각도변 사이 각
                angle = std::atan2(dy, dx);
                // 라이단 변환 각도로
                angle = angle * 180.0f / M_PI;

                auto angle_msg = std_msgs::msg::Float32();
                angle_msg.data = angle;
                yellow_angle_pub_->publish(angle_msg);

                yellow_line_points_pub_->publish(line_points_msg);

                float max_length = 0;
                int max_idx = 0;
                for (int i = 0; i < 4; i++) // 최대 길이 선으로 표현
                {
                    float length = cv::norm(vertices[i] - vertices[(i + 1) % 4]);
                    if (length > max_length)
                    {
                        max_length = length;
                        max_idx = i;
                    }
                }
                cv::line(line_display, vertices[max_idx], vertices[(max_idx + 1) % 4],
                         cv::Scalar(0, 255, 255), 2);
            }
        }

        // 흰색 선 그리기 (노란색과 같음)
        for (const auto &contour : white_contours)
        {
            double area = cv::contourArea(contour);
            if (area > 150.0)
            {
                white_line_detected = true;
                white_line_count++;

                cv::Moments moments = cv::moments(contour);
                if (moments.m00 != 0)
                {
                    white_line_x = moments.m10 / moments.m00;

                    white_line_x = (white_line_x / width) * 2 - 1;

                    auto white_pos_msg = std_msgs::msg::Float32();
                    white_pos_msg.data = white_line_x;
                    white_pos_pub_->publish(white_pos_msg);
                }

                std::vector<cv::Point> approx;
                cv::approxPolyDP(contour, approx, 10, true);

                cv::RotatedRect rot_rect = cv::minAreaRect(contour);
                cv::Point2f vertices[4];
                rot_rect.points(vertices);

                auto line_points_msg = std_msgs::msg::Float32MultiArray();
                line_points_msg.data.resize(8);
                for (int i = 0; i < 4; i++)
                {
                    float x = vertices[i].x;
                    float y = vertices[i].y;

                    line_points_msg.data[i * 2] = x;
                    line_points_msg.data[i * 2 + 1] = y;
                }

                float angle = 0.0f;
                float dx = vertices[3].x - vertices[0].x;
                float dy = vertices[3].y - vertices[0].y;

                angle = std::atan2(dy, dx); // 각도

                angle = angle * 180.0f / M_PI;

                auto angle_msg = std_msgs::msg::Float32();
                angle_msg.data = angle;
                white_angle_pub_->publish(angle_msg);

                white_line_points_pub_->publish(line_points_msg);

                float max_length = 0;
                int max_idx = 0;
                for (int i = 0; i < 4; i++)
                {
                    float length = cv::norm(vertices[i] - vertices[(i + 1) % 4]);
                    if (length > max_length)
                    {
                        max_length = length;
                        max_idx = i;
                    }
                }
                cv::line(line_display, vertices[max_idx], vertices[(max_idx + 1) % 4],
                         cv::Scalar(255, 255, 255), 2);
            }
        }

        if (yellow_line_detected || white_line_detected) // 노란선 감지 또는 흰 선감지
        {
            RCLCPP_INFO(this->get_logger(),
                        "Lines detected - Yellow: %d (count: %d), White: %d (count: %d)",
                        yellow_line_detected, yellow_line_count,
                        white_line_detected, white_line_count);
        }

        // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        // 차단바 검출
        cv::Mat bar_roi_mask = cv::Mat::zeros(resized_frame.size(), CV_8UC1);
        std::vector<cv::Point> bar_roi_points;
        for (int i = 0; i < 4; i++)
        {
            bar_roi_points.push_back(cv::Point(bar_vertices[i].x, bar_vertices[i].y));
        }
        cv::fillConvexPoly(bar_roi_mask, bar_roi_points, cv::Scalar(255));

        // 차단바용 색상 검출
        cv::Mat bar_hsv;
        cv::cvtColor(resized_frame, bar_hsv, cv::COLOR_BGR2HSV);

        // 차단바 검출을 위한 노란색 마스크
        cv::Mat bar_yellow_roi;
        cv::Scalar barrier_lower_yellow(15, 100, 100);
        cv::Scalar barrier_upper_yellow(35, 255, 255);
        cv::inRange(bar_hsv, barrier_lower_yellow, barrier_upper_yellow, bar_yellow_roi);

        // 차단바 영역에서 라인 검출 추가
        cv::Mat bar_yellow_line_mask, bar_white_line_mask;
        cv::inRange(bar_hsv, barrier_lower_yellow, barrier_upper_yellow, bar_yellow_line_mask);
        cv::inRange(bar_hsv, lower_white_hsv, upper_white_hsv, bar_white_line_mask);

        // ROI 적용
        cv::bitwise_and(bar_yellow_line_mask, bar_roi_mask, bar_yellow_line_mask);
        cv::bitwise_and(bar_white_line_mask, bar_roi_mask, bar_white_line_mask);

        // 노이즈 제거
        cv::morphologyEx(bar_yellow_line_mask, bar_yellow_line_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(bar_yellow_line_mask, bar_yellow_line_mask, cv::MORPH_CLOSE, kernel_large);
        cv::morphologyEx(bar_white_line_mask, bar_white_line_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(bar_white_line_mask, bar_white_line_mask, cv::MORPH_CLOSE, kernel_large);

        // ROI 적용
        cv::bitwise_and(bar_yellow_roi, bar_roi_mask, bar_yellow_roi);

        // 노이즈 제거
        cv::morphologyEx(bar_yellow_roi, bar_yellow_roi, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(bar_yellow_roi, bar_yellow_roi, cv::MORPH_CLOSE, kernel_large);

        // 라인 컨투어 검출
        std::vector<std::vector<cv::Point>> bar_yellow_line_contours, bar_white_line_contours;
        cv::findContours(bar_yellow_line_mask, bar_yellow_line_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(bar_white_line_mask, bar_white_line_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool barrier_yellow_detected = false;
        bool barrier_white_detected = false;
        float barrier_yellow_angle = 0.0f;
        float barrier_white_angle = 0.0f;

        // 차단바 노란색 라인 검출 처리
        for (const auto &contour : bar_yellow_line_contours)
        {
            double area = cv::contourArea(contour);
            if (area > 200.0)
            {
                barrier_yellow_detected = true;
                // 모멘트 계산
                cv::Moments moments = cv::moments(contour);
                if (moments.m00 != 0)
                {
                    float x = moments.m10 / moments.m00;
                    x = (x / width) * 2 - 1;
                }

                // 각도 계산을 위한 최소 영역 사각형
                cv::RotatedRect rot_rect = cv::minAreaRect(contour);

                // 각도 계산
                float width = rot_rect.size.width;
                float height = rot_rect.size.height;
                barrier_yellow_angle = rot_rect.angle;

                // width가 height보다 작을 때 90도 보정
                if (width < height)
                {
                    barrier_yellow_angle += 90.0f;
                }

                // 각도 범위를 -90 ~ 90으로 조정
                if (barrier_yellow_angle > 90.0f)
                {
                    barrier_yellow_angle -= 180.0f;
                }
                else if (barrier_yellow_angle < -90.0f)
                {
                    barrier_yellow_angle += 180.0f;
                }

                // 라인 그리기
                cv::Point2f vertices[4];
                rot_rect.points(vertices);
                float max_length = 0;
                int max_idx = 0;
                for (int i = 0; i < 4; i++)
                {
                    float length = cv::norm(vertices[i] - vertices[(i + 1) % 4]);
                    if (length > max_length)
                    {
                        max_length = length;
                        max_idx = i;
                    }
                }
                cv::line(line_display, vertices[max_idx], vertices[(max_idx + 1) % 4],
                         cv::Scalar(0, 255, 255), 2);
            }
        }

        // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        // 흰색 라인 처리
        for (const auto &contour : bar_white_line_contours)
        {
            double area = cv::contourArea(contour);
            if (area > 150.0)
            {
                barrier_white_detected = true;
                // 모멘트 계산
                cv::Moments moments = cv::moments(contour);
                if (moments.m00 != 0)
                {
                    float x = moments.m10 / moments.m00;
                    x = (x / width) * 2 - 1;
                }

                // 각도 계산을 위한 최소 영역 사각형
                cv::RotatedRect rot_rect = cv::minAreaRect(contour);

                // 각도 계산
                float width = rot_rect.size.width;
                float height = rot_rect.size.height;
                barrier_white_angle = rot_rect.angle;

                // width가 height보다 작을 때 90도 보정
                if (width < height)
                {
                    barrier_white_angle += 90.0f;
                }

                // 각도 범위를 -90 ~ 90으로 조정, 왼쪽이 - 오른쪽이 +
                if (barrier_white_angle > 90.0f)
                {
                    barrier_white_angle -= 180.0f;
                }
                else if (barrier_white_angle < -90.0f)
                {
                    barrier_white_angle += 180.0f;
                }

                // 라인 그리기
                cv::Point2f vertices[4];
                rot_rect.points(vertices);
                float max_length = 0;
                int max_idx = 0;
                for (int i = 0; i < 4; i++)
                {
                    float length = cv::norm(vertices[i] - vertices[(i + 1) % 4]);
                    if (length > max_length)
                    {
                        max_length = length;
                        max_idx = i;
                    }
                }
                cv::line(line_display, vertices[max_idx], vertices[(max_idx + 1) % 4],
                         cv::Scalar(255, 255, 255), 2);
            }
        }

        // 차단바 검출을 위한 컨투어 처리
        std::vector<std::vector<cv::Point>> bar_yellow_contours;
        cv::findContours(bar_yellow_roi, bar_yellow_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::RotatedRect> candidate_rects;
        barrier_detected = false;

        // 먼저 적절한 크기의 모든 사각형을 수집
        for (const auto &yellow_contour : bar_yellow_contours)
        {
            double area = cv::contourArea(yellow_contour);
            if (area > 50.0 && area < 1000.0)
            {
                cv::RotatedRect yellow_rect = cv::minAreaRect(yellow_contour);
                candidate_rects.push_back(yellow_rect);
            }
        }

        if (candidate_rects.size() >= 4)
        {
            // y 좌표로 정렬, 일직선상의 사각형들이 4개 이상
            std::sort(candidate_rects.begin(), candidate_rects.end(),
                      [](const cv::RotatedRect &a, const cv::RotatedRect &b)
                      {
                          return a.center.y < b.center.y;
                      });

            float prev_y = candidate_rects[0].center.y;
            int same_line_count = 1;
            float total_width = 0;
            float avg_height = 0;

            for (size_t i = 1; i < candidate_rects.size(); i++)
            {
                float curr_y = candidate_rects[i].center.y;
                if (std::abs(curr_y - prev_y) < 15.0)
                {
                    same_line_count++;
                    float width = std::min(candidate_rects[i].size.width, candidate_rects[i].size.height);
                    float height = std::max(candidate_rects[i].size.width, candidate_rects[i].size.height);
                    total_width += width;
                    avg_height += height;

                    float x_diff = std::abs(candidate_rects[i].center.x - candidate_rects[i - 1].center.x);
                    if (x_diff > 50.0)
                        continue;
                }
            }

            avg_height /= same_line_count;

            if (same_line_count >= 4 && avg_height > 10.0 && total_width > 50.0)
            {
                barrier_detected = true;

                // 시각화
                for (const auto &rect : candidate_rects)
                {
                    cv::Point2f vertices[4];
                    rect.points(vertices);
                    for (int i = 0; i < 4; i++)
                    {
                        cv::line(line_display, vertices[i], vertices[(i + 1) % 4],
                                 cv::Scalar(0, 0, 255), 2);
                    }
                }
            }
        }

        auto barrier_yellow_detected_msg = std_msgs::msg::Bool();
        barrier_yellow_detected_msg.data = barrier_yellow_detected;
        barrier_yellow_line_detected_pub_->publish(barrier_yellow_detected_msg);

        // 흰색 라인 검출 결과 publish
        auto barrier_white_detected_msg = std_msgs::msg::Bool();
        barrier_white_detected_msg.data = barrier_white_detected;
        barrier_white_line_detected_pub_->publish(barrier_white_detected_msg);

        // 노란색 라인 각도 publish (라인이 검출된 경우에만)
        if (barrier_yellow_detected)
        {
            auto barrier_yellow_angle_msg = std_msgs::msg::Float32();
            barrier_yellow_angle_msg.data = barrier_yellow_angle;
            barrier_yellow_line_angle_pub_->publish(barrier_yellow_angle_msg);
        }

        // 흰색 라인 각도 publish (라인이 검출된 경우에만)
        if (barrier_white_detected)
        {
            auto barrier_white_angle_msg = std_msgs::msg::Float32();
            barrier_white_angle_msg.data = barrier_white_angle;
            barrier_white_line_angle_pub_->publish(barrier_white_angle_msg);
        }
        // 차단바 검출 결과 펍
        auto barrier_msg = std_msgs::msg::Bool();
        barrier_msg.data = barrier_detected;
        barrier_detected_pub_->publish(barrier_msg);

        // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        // 표지판 ROI 마스크 생성
        cv::Mat sign_roi_mask = cv::Mat::zeros(resized_frame.size(), CV_8UC1);
        std::vector<cv::Point> roi_points;
        for (int i = 0; i < 4; i++)
        {
            roi_points.push_back(cv::Point(signs_vertices[i].x, signs_vertices[i].y));
        }
        cv::fillConvexPoly(sign_roi_mask, roi_points, cv::Scalar(255));

        // 먼저 ROI 영역만 추출
        cv::Mat roi_image;
        resized_frame.copyTo(roi_image, sign_roi_mask);

        // ROI 영역에 대해서만 HSV 변환 수행
        cv::Mat sign_hsv;
        cv::cvtColor(roi_image, sign_hsv, cv::COLOR_BGR2HSV);

        // 파란색 마스크 생성 - HSV 값 조정
        cv::Mat blue_mask;
        cv::Scalar lower_blue_hsv(100, 70, 50);
        cv::Scalar upper_blue_hsv(130, 255, 255);
        cv::inRange(sign_hsv, lower_blue_hsv, upper_blue_hsv, blue_mask);

        // 노이즈 제거
        cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel_large);

        // 파란색 영역 검출
        std::vector<std::vector<cv::Point>> blue_contours;
        cv::findContours(blue_mask, blue_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool blue_sign_detected = false;
        for (const auto &contour : blue_contours)
        {
            double area = cv::contourArea(contour);
            if (area > 100.0)
            {
                blue_sign_detected = true;
                break;
            }
        }

        // 파란색 표지판 검출 결과 발행
        auto blue_sign_msg = std_msgs::msg::Bool();
        blue_sign_msg.data = blue_sign_detected;
        blue_sign_detected_pub_->publish(blue_sign_msg);

        // ROI 영역 표시
        for (int i = 0; i < 4; i++)
        {
            cv::line(resized_frame, bar_vertices[i], bar_vertices[(i + 1) % 4],
                     cv::Scalar(255, 0, 0), 2);
        }
        for (int i = 0; i < 4; i++)
        {
            cv::line(resized_frame, signs_vertices[i], signs_vertices[(i + 1) % 4],
                     cv::Scalar(0, 0, 255), 2);
        }

        for (int i = 0; i < 4; i++)
        {
            cv::line(resized_frame, src_vertices[i], src_vertices[(i + 1) % 4],
                     cv::Scalar(0, 255, 0), 2);
        }

        // 표지판 ROI 영역 표시 (빨간색으로 표시)
        for (int i = 0; i < 4; i++)
        {
            cv::line(resized_frame, signs_vertices[i], signs_vertices[(i + 1) % 4],
                     cv::Scalar(0, 0, 255), 2);
        }

        yellow_line_valid = isLineValid(yellow_detection_array, yellow_line_detected);
        white_line_valid = isLineValid(white_detection_array, white_line_detected);

        array_index = (array_index + 1) % ARRAY_SIZE;

        auto yellow_detected_msg = std_msgs::msg::Bool();
        auto white_detected_msg = std_msgs::msg::Bool();

        yellow_detected_msg.data = yellow_line_valid;
        white_detected_msg.data = white_line_valid;

        yellow_detected_pub_->publish(yellow_detected_msg);
        white_detected_pub_->publish(white_detected_msg);

        sensor_msgs::msg::Image::SharedPtr original_msg =
            cv_bridge::CvImage(msg->header, "bgr8", resized_frame).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr yellow_mask_msg =
            cv_bridge::CvImage(msg->header, "mono8", yellow_mask_combined).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr white_mask_msg =
            cv_bridge::CvImage(msg->header, "mono8", white_mask_combined).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr line_msg =
            cv_bridge::CvImage(msg->header, "bgr8", line_display).toImageMsg();

        original_pub_->publish(*original_msg);
        yellow_mask_pub_->publish(*yellow_mask_msg);
        white_mask_pub_->publish(*white_mask_msg);
        line_pub_->publish(*line_msg);

        cv::imshow("Original Image", resized_frame);
        cv::imshow("Preprocessed", preprocessed);
        cv::imshow("Yellow Mask", yellow_mask_combined);
        cv::imshow("White Mask", white_mask_combined);
        cv::imshow("Detected Lines", line_display);
        //cv::imshow("Barrier Yellow", bar_yellow_roi);
        //cv::imshow("Barrier Line Yellow", bar_yellow_line_mask);
        //cv::imshow("Barrier Line White", bar_white_line_mask);
        cv::imshow("BLUe", blue_mask);
        cv::waitKey(1);
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}