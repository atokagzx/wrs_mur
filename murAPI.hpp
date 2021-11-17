//
// Created by Vladislav Bolotov on 12/6/16.
//

#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <atomic>
#include <chrono>
#include <thread>
#include <array>
#include <mutex>
#include <cstdint>
#include <vector>
#include <cstring>

#ifdef __linux__ 
#include <signal.h>
#endif

extern "C" {
#include <zmq.h>
}

namespace prvt {
    static inline int8_t checkPower(int power) {
        if (power > 100) {
            return 100;
        } 
        else if (power < -100) {
            return -100;
        }
        return power;
    }
#pragma pack (push, 1)
    struct ThrustersData {
        int8_t thruster_1 = 0;
        int8_t thruster_2 = 0;
        int8_t thruster_3 = 0;
        int8_t thruster_4 = 0;
        int8_t shoot = 0;
        int8_t drop = 0;
    };
#pragma pack (pop)

#pragma pack (push, 1)
    union McuData {
        enum {
            BINARY_DATA_SIZE = 36
        };
        struct {
            uint8_t thruster_1;  //1
            uint8_t thruster_2;  //2
            uint8_t thruster_3;  //3
            uint8_t thruster_4;  //4
            uint8_t button;      //5
            uint8_t i2c_1;       //6
            uint8_t i2c_2;       //7
            uint8_t battery;     //8
            float pitch;         //12
            float roll;          //16
            float yaw;           //20
            float i2c_1_value_1; //24
            float i2c_1_value_2; //28
            float i2c_2_value_1; //32
            float i2c_2_value_2; //36
        } data;

        struct {
            float x;
            float y;
            float z;
        } raw_mag;

        uint8_t binary[BINARY_DATA_SIZE];
    };
#pragma pack (pop)
}

inline void sleepFor(long long int time) {
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
}

struct Object {
    enum {
        TRIANGLE = 0,
        RECTANGLE = 1,
        CIRCLE = 2,
        NONE = 3
    };
    int x = -1;
    int y = -1;
    int r = -1;
    float angle = -1.0F;
    float size = -1.0F;
    int type = NONE;
};

class Timer {
public:
    void start();

    void stop();

    long long int elapsed();

    bool isStarted();

private:
    std::chrono::steady_clock::time_point m_start;
    bool m_isStarted = false;
};

class RobotPrivate {
public:
    RobotPrivate();

    ~RobotPrivate();

    friend class Robot;

private:
    std::atomic<bool> m_isRunningPortsThread;
    std::atomic<bool> m_isRunningCamerasThread;

    std::mutex m_transferGuard;
    std::mutex m_cameraGuard;

    prvt::ThrustersData m_thrustersData;
    prvt::McuData m_mcuData;

    std::function<Object(cv::Mat)> m_triangleDetector;
    std::function<Object(cv::Mat)> m_rectangleDetector;
    std::function<Object(cv::Mat)> m_circleDetector;

    const char *m_inPipe = "tcp://127.0.0.1:3390";
    const char *m_outPipe = "tcp://127.0.0.1:3391";
    const char *m_imageBottomPipe = "tcp://127.0.0.1:1771";
    const char *m_imageFrontPipe = "tcp://127.0.0.1:1772";

    void *m_zmqContext;
    void *m_subscriberMcuData;
    void *m_thrusterPusher;
    void *m_bottomImageSub;
    void *m_frontImageSub;

    std::vector<Object> m_detectedObjectsOne;
    std::vector<Object> m_detectedObjectsTwo;

    cv::Mat m_imageOne;
    cv::Mat m_imageTwo;
    cv::Mat m_simulatorFront;
    cv::Mat m_simulatorBottom;

    std::vector<int> m_detectrosListOne;
    std::vector<int> m_detectrosListTwo;

    cv::VideoCapture m_capOne;
    cv::VideoCapture m_capTwo;
};


class Robot {
public:

    static Robot &instance();

    static void init();

    static void setPortA(int val);

    static void setPortB(int val);

    static void setPortC(int val);

    static void setPortD(int val);

    static void setPorts(int a, int b, int c, int d);

    static float getInputAOne();

    static float getInputATwo();

    static float getInputBOne();

    static float getInputBTwo();

    static float getYaw();

    static float getRoll();

    static float getPitch();

    static void shoot();

    static void drop();

    static float getTemperature();

    static void initCamera(int id);

    static cv::Mat getCameraOneFrame();

    static cv::Mat getCameraTwoFrame();

    static void addDetectorToList(int detector, int camera);

    static void removeDetectorFromList(int detector, int camera);

    static std::vector<Object> getDetectedObjectsList(int camera);

    ~Robot();

private:

    Robot();

    static void onExit(int code);

    Robot &operator=(const Robot &rs);

    Robot(const Robot &rs);

    static std::shared_ptr<Robot> m_instance;
    static std::once_flag m_singleInstance;
    static std::shared_ptr<RobotPrivate> m_p;

};

#define mur Robot::instance()
//==========================================//

std::shared_ptr<RobotPrivate> Robot::m_p = nullptr;
std::once_flag Robot::m_singleInstance;
std::shared_ptr<Robot> Robot::m_instance = nullptr;

Object detectRectangle(cv::Mat &img) {
    static cv::Mat image;
    static std::vector<std::vector<cv::Point> > contours;
    static std::vector<cv::Point> hull;

    if (img.empty()) {
    	return Object();
    }

    image = img.clone();
    cv::cvtColor(image, image, CV_BGR2GRAY);
    cv::GaussianBlur(image, image, cv::Size(5, 5), 2);
    double cannyParams = cv::threshold(image, image, 0, 255, CV_THRESH_BINARY_INV + CV_THRESH_OTSU);
    cv::Canny(image, image, cannyParams, cannyParams / 2.0F);
    cv::findContours(image, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    Object objectToRet;
    for (std::size_t i = 0; i < contours.size(); i++) {
        if (contours.at(i).size() < 5) {
            continue;
        }
        if (std::fabs(cv::contourArea(contours.at(i))) < 300.0) {
            continue;
        }
        cv::RotatedRect bEllipse = cv::fitEllipse(contours.at(i));
        cv::convexHull(contours.at(i), hull, true);
        cv::approxPolyDP(hull, hull, 15, true);
        if (!cv::isContourConvex(hull)) {
            continue;
        }
        if (hull.size() == 4) {
            objectToRet.x = (int) bEllipse.center.x;
            objectToRet.y = (int) bEllipse.center.y;
            objectToRet.r = cv::boundingRect(contours.at(i)).width;
            objectToRet.angle = bEllipse.angle;
            objectToRet.size = bEllipse.size.area();
            objectToRet.type = Object::RECTANGLE;
            return objectToRet;
        }
    }
    return Object();
}

Object detectTriangle(cv::Mat &img) {
    static cv::Mat image;
    static std::vector<std::vector<cv::Point> > contours;
    static std::vector<cv::Point> hull;

    if (img.empty()) {
    	return Object();
    }

    image = img.clone();
    cv::cvtColor(image, image, CV_BGR2GRAY);
    cv::GaussianBlur(image, image, cv::Size(5, 5), 2);
    double cannyParams = cv::threshold(image, image, 0, 255, CV_THRESH_BINARY_INV + CV_THRESH_OTSU);
    cv::Canny(image, image, cannyParams, cannyParams / 2.0F);
    cv::findContours(image, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    Object objectToRet;
    for (std::size_t i = 0; i < contours.size(); i++) {
        if (contours.at(i).size() < 5) {
            continue;
        }
        if (std::fabs(cv::contourArea(contours.at(i))) < 300.0) {
            continue;
        }
        cv::RotatedRect bEllipse = cv::fitEllipse(contours.at(i));
        cv::convexHull(contours.at(i), hull, true);
        cv::approxPolyDP(hull, hull, 15, true);
        if (!cv::isContourConvex(hull)) {
            continue;
        }
        if (hull.size() == 3) {
            objectToRet.x = (int) bEllipse.center.x;
            objectToRet.y = (int) bEllipse.center.y;
            objectToRet.r = cv::boundingRect(contours.at(i)).width;
            objectToRet.angle = bEllipse.angle;
            objectToRet.size = bEllipse.size.area();
            objectToRet.type = Object::TRIANGLE;
            return objectToRet;
        }
    }
    return Object();
}

Object detectCircle(cv::Mat &img) {

    if (img.empty()) {
    	return Object();
    }

    static cv::Mat image;
    static std::vector<std::vector<cv::Point> > contours;
    static std::vector<cv::Point> hull;
    image = img.clone();
    cv::cvtColor(image, image, CV_BGR2GRAY);
    cv::GaussianBlur(image, image, cv::Size(5, 5), 2);
    double cannyParams = cv::threshold(image, image, 0, 255, CV_THRESH_BINARY_INV + CV_THRESH_OTSU);
    cv::Canny(image, image, cannyParams, cannyParams / 2.0F);
    cv::findContours(image, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    Object objectToRet;
    for (std::size_t i = 0; i < contours.size(); i++) {
        if (contours.at(i).size() < 5) {
            continue;
        }
        if (std::fabs(cv::contourArea(contours.at(i))) < 300.0) {
            continue;
        }
        cv::RotatedRect bEllipse = cv::fitEllipse(contours.at(i));
        cv::convexHull(contours.at(i), hull, true);
        cv::approxPolyDP(hull, hull, 15, true);
        if (!cv::isContourConvex(hull)) {
            continue;
        }

        double area = cv::contourArea(contours.at(i));
        cv::Rect r = cv::boundingRect(contours.at(i));
        double radius = r.width / 2.0;

        if (std::abs(1.0 - ((double) r.width / (double) r.height)) <= 0.2 &&
            std::abs(1.0 - (area / (CV_PI * std::pow(radius, 2.0)))) <= 0.2) {
            objectToRet.x = (int) bEllipse.center.x;
            objectToRet.y = (int) bEllipse.center.y;
            objectToRet.r = (int) radius;
            objectToRet.angle = bEllipse.angle;
            objectToRet.size = bEllipse.size.area();
            objectToRet.type = Object::CIRCLE;
            return objectToRet;
        }
    }
    return Object();
}

long long int Timer::elapsed() {
    return m_isStarted ?
           std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_start).count()
                       : 0;
}

void Timer::start() {
    m_start = std::chrono::steady_clock::now();
    m_isStarted = true;
}

void Timer::stop() {
    m_isStarted = false;
}

bool Timer::isStarted() {
    return m_isStarted;
}

RobotPrivate::RobotPrivate() :
        m_isRunningPortsThread(true),
        m_isRunningCamerasThread(true),
        m_zmqContext(zmq_ctx_new()),
        m_subscriberMcuData(zmq_socket(m_zmqContext, ZMQ_SUB)),
        m_thrusterPusher(zmq_socket(m_zmqContext, ZMQ_PAIR)),
        m_bottomImageSub(zmq_socket(m_zmqContext, ZMQ_SUB)),
        m_frontImageSub(zmq_socket(m_zmqContext, ZMQ_SUB)) {

    int hwmOption = 5;
    zmq_setsockopt(m_subscriberMcuData, ZMQ_SNDHWM, &hwmOption, sizeof(int));
    zmq_setsockopt(m_subscriberMcuData, ZMQ_SUBSCRIBE, 0, 0);

    zmq_setsockopt(m_bottomImageSub, ZMQ_SUBSCRIBE, 0, 0);
    zmq_setsockopt(m_bottomImageSub, ZMQ_SNDHWM, &hwmOption, sizeof(int));

    zmq_setsockopt(m_frontImageSub, ZMQ_SUBSCRIBE, 0, 0);
    zmq_setsockopt(m_frontImageSub, ZMQ_SNDHWM, &hwmOption, sizeof(int));


    int timeoutOption = 3000;
    zmq_setsockopt(m_thrusterPusher, ZMQ_SNDTIMEO, &timeoutOption, sizeof(int));

    zmq_connect(m_subscriberMcuData, m_inPipe);
    zmq_connect(m_bottomImageSub, m_imageBottomPipe);
    zmq_connect(m_frontImageSub, m_imageFrontPipe);

    zmq_connect(m_thrusterPusher, m_outPipe);

    auto update = [this]() {
        while (m_isRunningPortsThread) {
            {
                std::lock_guard<std::mutex> loc(m_transferGuard);

                zmq_msg_t mcuInfo;
                zmq_msg_init(&mcuInfo);
                if (-1 != zmq_msg_recv(&mcuInfo, m_subscriberMcuData, ZMQ_NOBLOCK)) {
                    if (zmq_msg_size(&mcuInfo) == sizeof(prvt::McuData)) {
                        std::memcpy(&m_mcuData, zmq_msg_data(&mcuInfo), prvt::McuData::BINARY_DATA_SIZE);
                    }
                }
                zmq_msg_close(&mcuInfo);

                zmq_msg_t thrustersData;
                zmq_msg_init_size(&thrustersData, sizeof(prvt::ThrustersData));
                std::memcpy(zmq_msg_data(&thrustersData), &m_thrustersData, sizeof(prvt::ThrustersData));
                if (-1 == zmq_msg_send(&thrustersData, m_thrusterPusher, 0)) {
                    std::cerr << "Unable to send thrusters data to server/simulator" << std::endl;
                }
                zmq_msg_close(&thrustersData);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };
    std::thread updateThread(update);
    updateThread.detach();

    auto camerasUpdate = [this]() {
#ifndef __linux__ 
        while (m_isRunningCamerasThread) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            zmq_msg_t frameBottom;
            zmq_msg_init(&frameBottom);
            if (-1 != zmq_msg_recv(&frameBottom, m_bottomImageSub, ZMQ_NOBLOCK)) {
                std::lock_guard<std::mutex> loc(m_cameraGuard);
                std::vector<char> data((char*)zmq_msg_data(&frameBottom),(char*)zmq_msg_data(&frameBottom) + zmq_msg_size(&frameBottom));
                m_imageOne = cv::imdecode(cv::Mat(data), 1);
            }
            zmq_msg_close(&frameBottom);

            zmq_msg_t frameFront;
            zmq_msg_init(&frameFront);
            if (-1 != zmq_msg_recv(&frameFront, m_frontImageSub, ZMQ_NOBLOCK)) {
                std::lock_guard<std::mutex> loc(m_cameraGuard);
                std::vector<char> data((char*)zmq_msg_data(&frameFront),(char*)zmq_msg_data(&frameFront) + zmq_msg_size(&frameFront));
                m_imageTwo = cv::imdecode(cv::Mat(data), 1);
            }
            zmq_msg_close(&frameFront);
            {
                std::lock_guard<std::mutex> loc(m_cameraGuard);
                m_detectedObjectsOne.clear();

                for (const auto &item : m_detectrosListOne) {

                    if (item == Object::TRIANGLE) {
                        m_detectedObjectsOne.push_back(detectTriangle(m_imageOne));
                    }

                    if (item == Object::RECTANGLE) {
                        m_detectedObjectsOne.push_back(detectRectangle(m_imageOne));
                    }

                    if (item == Object::CIRCLE) {
                        m_detectedObjectsOne.push_back(detectCircle(m_imageOne));
                    }
                }            

                m_detectedObjectsTwo.clear();
                for (const auto &item : m_detectrosListTwo) {
                    if (item == Object::TRIANGLE) {
                        m_detectedObjectsTwo.push_back(detectTriangle(m_imageTwo));
                    }

                    if (item == Object::RECTANGLE) {
                        m_detectedObjectsTwo.push_back(detectRectangle(m_imageTwo));
                    }

                    if (item == Object::CIRCLE) {
                        m_detectedObjectsTwo.push_back(detectCircle(m_imageTwo));
                    }
                }
            }
        }
#endif
#ifdef __linux__
        while (m_isRunningCamerasThread) {
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::lock_guard<std::mutex> loc(m_cameraGuard);
            if (m_capOne.isOpened()) {
                m_detectedObjectsOne.clear();
                m_capOne >> m_imageOne;

                for (const auto &item : m_detectrosListOne) {

                    if (item == Object::TRIANGLE) {
                        m_detectedObjectsOne.push_back(detectTriangle(m_imageOne));
                    }

                    if (item == Object::RECTANGLE) {
                        m_detectedObjectsOne.push_back(detectRectangle(m_imageOne));
                    }

                    if (item == Object::CIRCLE) {
                        m_detectedObjectsOne.push_back(detectCircle(m_imageOne));
                    }
                }
            }

            if (m_capTwo.isOpened()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                m_detectedObjectsTwo.clear();
                m_capTwo >> m_imageTwo;
                for (const auto &item : m_detectrosListTwo) {
                    if (item == Object::TRIANGLE) {
                        m_detectedObjectsTwo.push_back(detectTriangle(m_imageTwo));
                    }

                    if (item == Object::RECTANGLE) {
                        m_detectedObjectsTwo.push_back(detectRectangle(m_imageTwo));
                    }

                    if (item == Object::CIRCLE) {
                        m_detectedObjectsTwo.push_back(detectCircle(m_imageTwo));
                    }
                }
            }
        }
    }
#endif
    };
        /*
    */

    std::thread camerasThread(camerasUpdate);
    camerasThread.detach();
}

RobotPrivate::~RobotPrivate() {
    m_isRunningPortsThread = false;
    m_isRunningCamerasThread = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    m_capOne.release();
    m_capTwo.release();

    zmq_close(m_subscriberMcuData);
    zmq_close(m_thrusterPusher);
    zmq_ctx_destroy(m_zmqContext);
}


Robot::Robot() {
    m_p.reset(new RobotPrivate);
#ifdef __linux__ 
    signal(SIGTERM, Robot::onExit);
#endif
}

Robot::~Robot() {

}

void Robot::onExit(int code) {
    m_p.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    exit(0);
}

Robot &Robot::instance() {
    std::call_once(Robot::m_singleInstance, []() {
        Robot::m_instance.reset(new Robot());
    });

    return *Robot::m_instance;
}

Robot::Robot(const Robot &rs) {
    m_instance = rs.m_instance;
}

Robot &Robot::operator=(const Robot &rs) {
    if (this != &rs) {
        m_instance = rs.m_instance;
    }
    return *this;
}


void Robot::setPortA(int val) {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    m_p->m_thrustersData.thruster_1 = prvt::checkPower(val);
}

float Robot::getTemperature() {
    return 0.0f;
}

void Robot::setPortB(int val) {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    m_p->m_thrustersData.thruster_2 = prvt::checkPower(val);
}

void Robot::setPortC(int val) {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    m_p->m_thrustersData.thruster_3 = prvt::checkPower(val);
}

void Robot::setPortD(int val) {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    m_p->m_thrustersData.thruster_4 = prvt::checkPower(val);
}

float Robot::getInputAOne() {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    return m_p->m_mcuData.data.i2c_1_value_1;
}

float Robot::getInputATwo() {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    return m_p->m_mcuData.data.i2c_1_value_2;
}

float Robot::getInputBOne() {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    return m_p->m_mcuData.data.i2c_2_value_1;
}

float Robot::getInputBTwo() {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    return m_p->m_mcuData.data.i2c_2_value_2;
}

cv::Mat Robot::getCameraOneFrame() {
    std::lock_guard<std::mutex> loc(m_p->m_cameraGuard);
    if (m_p->m_imageOne.empty()) {
        return cv::Mat::zeros(240, 320, CV_8UC3);
    }
    return m_p->m_imageOne.clone();
}

cv::Mat Robot::getCameraTwoFrame() {
    std::lock_guard<std::mutex> loc(m_p->m_cameraGuard);
        if (m_p->m_imageTwo.empty()) {
        return cv::Mat::zeros(240, 320, CV_8UC3);
    }
    return m_p->m_imageTwo.clone();
}

void Robot::setPorts(int a, int b, int c, int d) {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    m_p->m_thrustersData.thruster_1 = prvt::checkPower(a);
    m_p->m_thrustersData.thruster_2 = prvt::checkPower(b);
    m_p->m_thrustersData.thruster_3 = prvt::checkPower(c);
    m_p->m_thrustersData.thruster_4 = prvt::checkPower(d);


}

void Robot::initCamera(int id) {
    std::lock_guard<std::mutex> loc(m_p->m_cameraGuard);

    if (id == 0) {
        if (!m_p->m_capOne.isOpened()) {
            m_p->m_capOne.open(id);
            m_p->m_capOne.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
            m_p->m_capOne.set(CV_CAP_PROP_FRAME_WIDTH, 320);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if (!m_p->m_capOne.isOpened()) {
                std::cerr << "Unable to open camera with id: " << id << std::endl;
                return;
            }
        }
    }

    if (id == 1) {
        if (!m_p->m_capTwo.isOpened()) {
            m_p->m_capTwo.open(id);
            m_p->m_capTwo.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
            m_p->m_capTwo.set(CV_CAP_PROP_FRAME_WIDTH, 320);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if (!m_p->m_capTwo.isOpened()) {
                std::cerr << "Unable to open camera with id: " << id << std::endl;
                return;
            }
        }
    }
}

void Robot::addDetectorToList(int detector, int camera) {
    std::lock_guard<std::mutex> loc(m_p->m_cameraGuard);

    if (camera == 0) {
        for (auto &item : m_p->m_detectrosListOne) {
            if (item == detector) {
                return;
            }
        }
        m_p->m_detectrosListOne.push_back(detector);
    }

    if (camera == 1) {
        for (auto &item : m_p->m_detectrosListTwo) {
            if (item == detector) {
                return;
            }
        }

        m_p->m_detectrosListTwo.push_back(detector);
    }
}

void Robot::removeDetectorFromList(int detector, int camera) {
    std::lock_guard<std::mutex> loc(m_p->m_cameraGuard);
    if (camera == 0) {
        for (auto it = m_p->m_detectrosListOne.begin(); it != m_p->m_detectrosListOne.end(); ++it) {
            if (*it == detector) {
                m_p->m_detectrosListOne.erase(it);
                return;
            }
        }
    }

    if (camera == 1) {
        for (auto it = m_p->m_detectrosListTwo.begin(); it != m_p->m_detectrosListTwo.end(); ++it) {
            if (*it == detector) {
                m_p->m_detectrosListTwo.erase(it);
                return;
            }
        }
    }
}

std::vector<Object> Robot::getDetectedObjectsList(int camera) {
    std::vector<Object> vec = m_p->m_detectedObjectsOne;
    if (camera == 0) {
        vec = m_p->m_detectedObjectsOne;
    }
    if (camera == 1) {
        vec = m_p->m_detectedObjectsTwo;
    }
    return vec;
}

void Robot::shoot() {
    if (m_p->m_thrustersData.shoot == std::numeric_limits<int8_t>::max()) {
        m_p->m_thrustersData.shoot = 0;
    } else {
        m_p->m_thrustersData.shoot += 1;
    }
}

void Robot::drop() {
    if (m_p->m_thrustersData.drop == std::numeric_limits<int8_t>::max()) {
        m_p->m_thrustersData.drop = 0;
    } else {
        m_p->m_thrustersData.drop += 1;
    }
}

float Robot::getYaw() {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    return m_p->m_mcuData.data.yaw;
}

float Robot::getPitch() {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    return m_p->m_mcuData.data.pitch;
}

float Robot::getRoll() {
    std::lock_guard<std::mutex> loc(m_p->m_transferGuard);
    return m_p->m_mcuData.data.roll;
}


