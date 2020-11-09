#include "detectlane.h"

void swap(int *a, int *b)
{
    int temp = *a;
    *a = *b;
    *b = temp;
}
//tính độ dài đoạn thẳng
float lineLength(Vec4i line)
{
    return sqrt((line[0] - line[2]) * (line[0] - line[2]) + (line[1] - line[3]) * (line[1] - line[3]));
}
//tính góc nghiêng đường thẳng
float lineAngle(Vec4i line)
{
    float X = line[2] - line[0];
    float Y = line[3] - line[1];
    float angle = atan2(Y, X) * 180 / CV_PI;
    if (angle > 90)
    {
        angle -= 180;
    }
    if (angle < -90)
    {
        angle += 180;
    }
    return angle;
}
//tính khoảng cách giữa 2 điểm
float distance(float x1, float y1, float x2, float y2)
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
// lấy toạ độ x của hai điểm thuộc hai đường thẳng với cùng y

Point getPointInLine(Vec4i line, float y)
{
    return Point((y - line[1]) * (line[0] - line[2]) / (line[1] - line[3]) + line[0], y);
}
// Xác định xem hai vector có giao nhau hay không
bool isIntersect(Vec4i a, Vec4i b)
{
    int head1 = getPointInLine(a, Config::HEIGHT / 2).x;
    int head2 = getPointInLine(b, Config::HEIGHT / 2).x;
    int tail1 = getPointInLine(a, Config::HEIGHT).x;
    int tail2 = getPointInLine(b, Config::HEIGHT).x;
    return (head1 > head2 && tail1 < tail2) ||
            (head1 < head2 && tail1 > tail2) ||
            (abs(head1 - head2) < Config::LANE_WIDTH / 2);
}

struct Dist1
{
    bool operator()(Vec4i a, Vec4i b)
    {
        float angle1 = lineAngle(a);
        float angle2 = lineAngle(b);
        float dist = min(distance(a[0], a[1], b[0], b[1]), distance(a[2], a[3], b[2], b[3]));
        dist = min(dist, distance(a[0], a[1], b[2], b[3]));
        dist = min(dist, distance(a[2], a[3], b[0], b[1]));

        return (abs(angle1 - angle2) < 10) && (dist < 20);
    }
};

struct Dist2
{
    bool operator()(Vec4i a, Vec4i b)
    {
        float angle1 = lineAngle(a);
        float angle2 = lineAngle(b);
        float dist = min(distance(a[0], a[1], b[0], b[1]), distance(a[2], a[3], b[2], b[3]));
        dist = min(dist, distance(a[0], a[1], b[2], b[3]));
        dist = min(dist, distance(a[2], a[3], b[0], b[1]));

        return (abs(angle1 - angle2) < 5) && (dist < 50);
    }
};
// định nghĩa ra biến làn không xác định.

Vec4i DetectLane::UndefinedLine = Vec4i();

DetectLane::DetectLane()
{
    // khởi tạo ví trí xe ban đầu
    m_signDetected = None;
    m_carPos.x = Config::WIDTH / 2;
    m_carPos.y = Config::HEIGHT;

    m_preLane = Vec4i(Config::WIDTH / 2, Config::HEIGHT / 2, Config::WIDTH / 2, Config::HEIGHT);
}
// Tính góc lệch
float DetectLane::errorAngle(const Point &dst)
{
    if (dst.x == m_carPos.x)
        return 0;
    if (dst.y == m_carPos.y)
        return (dst.x < m_carPos.x ? -90 : 90);

    double dx = dst.x - m_carPos.x;
    double dy = m_carPos.y - dst.y;

    if (dx < 0)
        return -atan(-dx / dy) * 180 / CV_PI;
    return atan(dx / dy) * 180 / CV_PI;
}

// cập nhật hình ảnh mới vào để xử lý

DetectLane::~DetectLane() {}

Mat DetectLane::update(const Mat &src)
{
    Mat output = src.clone();
    // mặc định các làn sẽ được gán là không tồn tại.
    m_stopLane = UndefinedLine;
    m_leftLane = UndefinedLine;
    m_rightLane = UndefinedLine;
    // resize ảnh
    resize(output, output, Size(Config::WIDTH, Config::HEIGHT));

    Mat binary = preProcess(output);
    // Tiền xử lý, nhị phân hoá ảnh
    vector<Vec4i> laneLines;
    vector<Vec4i> stopLines;
    fitLane2Line(binary, laneLines, stopLines);
    findLane(laneLines);
    findSign(src);

    if (m_leftLane != UndefinedLine)
    {
        Point pt1 = getPointInLine(m_leftLane, Config::HEIGHT);
        Point pt2 = getPointInLine(m_leftLane, Config::HEIGHT / 2);
        line(output, pt1, pt2, Scalar(0, 0, 255), 3);
    }

    if (m_rightLane != UndefinedLine)
    {
        Point pt1 = getPointInLine(m_rightLane, Config::HEIGHT);
        Point pt2 = getPointInLine(m_rightLane, Config::HEIGHT / 2);
        line(output, pt1, pt2, Scalar(0, 255, 0), 3);
    }

    if (m_stopLane != UndefinedLine)
    {
        line(output, Point(m_stopLane[0], m_stopLane[1]), Point(m_stopLane[2], m_stopLane[3]), Scalar(255, 0, 0), 3);
    }

    return output;
}
// Tính góc lệch (hàm này lấy ra các điểm để truyền vào hàm errorAngle tính góc lệch)
float DetectLane::getErrorAngle(TrafficSign signSignal)
{
    Point dst(Config::WIDTH / 2, Config::HEIGHT / 3 * 2);
    int p1 = getPointInLine(m_leftLane, Config::HEIGHT / 3 * 2).x;
    int p2 = getPointInLine(m_rightLane, Config::HEIGHT / 3 * 2).x;
    // Nếu thấy cả hai làn thì tính tâm đường bằng trung điểm hai đầu
    if (m_leftLane != UndefinedLine && m_rightLane != UndefinedLine)
    {
        if (signSignal == TurnLeft)
        {
            dst.x = p1 + Config::LANE_WIDTH / 2;
        }
        else if (signSignal == TurnRight)
        {
            dst.x = p2 - Config::LANE_WIDTH / 2;
        }
        else
        {
            dst.x = (p1 + p2) / 2;
        }
    }
        // Nếu mất bên phải thì tính theo bên trái hoac bên phải
    else if (m_rightLane != UndefinedLine)
    {
        dst.x = p2 - Config::LANE_WIDTH / 2;
    }
    else if (m_leftLane != UndefinedLine)
    {
        dst.x = p1 + Config::LANE_WIDTH / 2;
    }

    m_preLane = Vec4i(dst.x, dst.y, m_carPos.x, m_carPos.y);

    return errorAngle(dst);
}
// Kiểm tra xe có nhận biết được làn đường nào hay không ?
bool DetectLane::HasLane()
{
    return m_leftLane != UndefinedLine || m_rightLane != UndefinedLine;
}
// Kiếm tra xem có phải vạch dừng lại hay không 

bool DetectLane::IsStop()
{
    return m_stopLane != UndefinedLine && (m_stopLane[1] + m_stopLane[3]) > Config::HEIGHT / 3 * 2;
}
// Kiếm tra xem có phải biên báo hay không 

TrafficSign DetectLane::GetSign()
{
    return m_signDetected;
}
// Tiền xử lý ảnh trả về ảnh nhị phân

Mat DetectLane::preProcess(const Mat &src)
{
    Mat binary;

    binary = binaryImage(src);

    return binary;
}
// xác định các đường thẳng xem có phải là làn đường hay không ?

void DetectLane::findLane(const vector<Vec4i> &lines)
{
    if (lines.size() == 0)
        return;

    vector<int> labels;

    int cnt = partition(lines, labels, Dist1());

    int countLine[cnt];
    Vec4i mean[cnt];

    for (int i = 0; i < cnt; i++)
    {
        countLine[i] = 0;
        mean[i] = Vec4i(0, 0, 0, 0);
    }

    for (int i = 0; i < labels.size(); i++)
    {
        countLine[labels[i]]++;
    }

    for (int i = 0; i < lines.size(); i++)
    {
        mean[labels[i]] += lines[i];
    }

    for (int i = 0; i < cnt; i++)
    {
        mean[i] /= countLine[i];
    }

    for (int i = 0; i < cnt - 1; i++)
    {
        for (int j = i + 1; j < cnt; j++)
        {
            if (countLine[i] < countLine[j])
            {
                swap(countLine[i], countLine[j]);
                swap(mean[i], mean[j]);
            }
        }
    }
    // Khởi tạo mặc định hai làn đường hai bên là không nhìn thấy.

    m_leftLane = UndefinedLine;
    m_rightLane = UndefinedLine;

    if (cnt >= 2)
    {
        m_leftLane = mean[0];

        for (int i = 1; i < cnt; i++)
        {
            if (!isIntersect(m_leftLane, mean[i]))
            {
                int x1 = getPointInLine(m_leftLane, Config::HEIGHT).x;
                int x2 = getPointInLine(mean[i], Config::HEIGHT).x;

                if ((Config::WIDTH / 2 >= x1 && Config::WIDTH / 2 <= x2) || ((Config::WIDTH / 2 <= x1 && Config::WIDTH / 2 >= x2)))
                {
                    m_rightLane = mean[i];
                    break;
                }
            }
        }
        if (m_rightLane != UndefinedLine && getPointInLine(m_leftLane, Config::HEIGHT).x > getPointInLine(m_rightLane, Config::HEIGHT).x)
        {
            std::swap(m_leftLane, m_rightLane);
        } else if (getPointInLine(m_leftLane, Config::HEIGHT).x > Config::WIDTH / 2)
        {
            m_rightLane = m_leftLane;
            m_leftLane = UndefinedLine;
        }
    }
    else if (cnt > 0)
    {
        if (getPointInLine(mean[0], Config::HEIGHT).x < Config::WIDTH / 2)
            m_leftLane = mean[0];
        else
            m_rightLane = mean[0];
    }
}
// Tìm biên báo

void DetectLane::findSign(const Mat &src)
{
    const int MinSignArea = 500;

    Mat img, imgHSV, signMask, signMask1, cleanSignMask, sign;
    img = src.clone();
    medianBlur(img, img, 3);
    cvtColor(img, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, Scalar(Config::BG_SIGN[0], Config::BG_SIGN[1], Config::BG_SIGN[2]),
            Scalar(Config::BG_SIGN[3], Config::BG_SIGN[4], Config::BG_SIGN[5]),
            signMask);
    inRange(imgHSV, Scalar(0, 40, 80), Scalar(5, 140, 140), signMask1);

    bitwise_or(signMask, signMask1, signMask);

    if (countNonZero(signMask) < MinSignArea)
    {
        m_signDetected = None;
        return;
    }

    std::vector<std::vector<Point>> contours;
    findContours(signMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
   // imshow("HSV Slider", signMask); Dung lenh nay de dieu chinh, quan sat va lay nguong HSV cho BG_SIGN va YELLOW_SIGN, sau khi lay duoc nguong thi dien vao file config
    if (contours.size() == 0)
    {
        m_signDetected = None;
        return;
    }
    float maxArea = 0;
    int contourIndex = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        Moments mm = moments(contours[i]);
        float cY = mm.m01 / mm.m00;
        if (cY >= Config::HEIGHT / 3) {
            float cntArea = contourArea(contours[i]);
            if (cntArea > maxArea)
            {
                maxArea = cntArea;
                contourIndex = i;
            }
        }
    }
    vector<vector<Point>> hulls(1);
    convexHull(contours[contourIndex], hulls[0]);
    cleanSignMask = Mat::zeros(signMask.size(), signMask.type());
    drawContours(cleanSignMask, hulls, 0, 255, CV_FILLED);
    cv::Rect boundRect = boundingRect(contours[contourIndex]);
    int signNumPixel = countNonZero(cleanSignMask);
    float ratio = (float) boundRect.width / boundRect.height;
    if (ratio < 0.7 || ratio > 1.3 || contourArea(contours[contourIndex]) < MinSignArea)
    {
        m_signDetected = None;
        return;
    }

    sign = Mat::zeros(imgHSV.size(), imgHSV.type());
    imgHSV.copyTo(sign, cleanSignMask);

    Mat blueSign;
    inRange(sign, Scalar(Config::BLUE_SIGN[0], Config::BLUE_SIGN[1], Config::BLUE_SIGN[2]),
            Scalar(Config::BLUE_SIGN[3], Config::BLUE_SIGN[4], Config::BLUE_SIGN[5]),
            blueSign);

    if (countNonZero(blueSign) > signNumPixel * 0.15)
    {
        m_signDetected = Change;
        return;
    }

    Mat yellowSign;
    inRange(sign, Scalar(Config::YELLOW_SIGN[0], Config::YELLOW_SIGN[1], Config::YELLOW_SIGN[2]),
            Scalar(Config::YELLOW_SIGN[3], Config::YELLOW_SIGN[4], Config::YELLOW_SIGN[5]),
            yellowSign);

    if (countNonZero(yellowSign) > signNumPixel * 0.15)
    {
        m_signDetected = Stop;
        return;
    }

    m_signDetected = None;
}

void DetectLane::fitLane2Line(const Mat &src, vector<Vec4i>& laneLines, vector<Vec4i>& stopLines)
{
    vector<Vec4i> lines;
        // Dùng HoughLine tìm ra các đường thẳng

    HoughLinesP(src, lines, 1, CV_PI / 180, 20, 10, 7);

    for (int i = 0; i < lines.size(); i++)
    { // Chỉ xét các đường nằm trên đường chân trời.
        if (lines[i][1] < Config::SKY_LINE)
        {
            if (lines[i][3] < Config::HEIGHT / 3 * 2) continue;
        }
        if (lines[i][3] < Config::SKY_LINE)
        {
            if (lines[i][1] < Config::HEIGHT / 3 * 2) continue;
        }

        float angle = lineAngle(lines[i]);

        if (abs(angle) < 15)
        {         // Chỉ xét các đường có góc nghiêng lớn hơn 15
            int weight = max(lineLength(lines[i]) / 10, 1.0f);
            for (int j = 0; j < weight; j++)
            {
                stopLines.push_back(lines[i]);
            }
            continue;
        }

        if (lines[i][1] < lines[i][3])
        {
            std::swap(lines[i][0], lines[i][2]);
            std::swap(lines[i][1], lines[i][3]);
        }

        int weight = max(lineLength(lines[i]) / 10, 1.0f);
        for (int j = 0; j < weight; j++)
        {
            laneLines.push_back(lines[i]);
        }
    }
}
// Nhị phân ảnh những vùng quan tâm có màu trắng ngược lại có màu đen

Mat DetectLane::binaryImage(const Mat &src)
{
    Mat img, imgThresholded, imgHSV, cannyImg, gray;

    img = src.clone();

    GaussianBlur(img, img, Size(3, 3), 0);
    // Đổi hệ màu

    cvtColor(img, imgHSV, COLOR_BGR2HSV);
        // Chuyển sang ảnh xám
    cvtColor(img, gray, COLOR_BGR2GRAY);
        // Lọc màu dùng hệ màu HSV

    inRange(imgHSV, Scalar(Config::LOW_H, Config::LOW_S, Config::LOW_V),
            Scalar(Config::HIGH_H, Config::HIGH_S, Config::HIGH_V),
            imgThresholded);
        // Khử nhiễu
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5), Point(2, 2));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element, Point(2, 2));
    dilate(imgThresholded, imgThresholded, element, Point(2, 2));

    rectangle(imgThresholded, Point(70, 220), Point(270, 240), 0, CV_FILLED);
    // Tách ra các đường cạnh
    Canny(gray, cannyImg, Config::CANNY_LOW,Config::CANNY_HIGH);

    Mat lane = Mat::zeros(img.size(), CV_8UC1);

    cannyImg.copyTo(lane, imgThresholded);

    if (Config::DEBUG) {
        imshow("Binary", imgThresholded);
    }

    return lane;
}
