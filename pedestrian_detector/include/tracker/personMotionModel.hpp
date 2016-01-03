#ifndef PERSONMOTIONMODEL_HPP
#define PERSONMOTIONMODEL_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>


using namespace cv;
using namespace std;

class PersonModel
{

    private:

    //Sampling time - not used yet
    ros::Time lastUpdate;

    double delta_t;

    int median_window;



    //Five last velocities (Fast Five. lol) - not used yet
    Point2d velocity[25];
    Point2d filteredVelocity;
    void updateVelocityArray(Point3d detectedPosition);

    public:

    int id;

    bool toBeDeleted;

    Point2d bbCenter;

    Mat bvtHistogram;

    bool lockedOnce;
    bool deadReckoning;
    //Increment this each time there is no detection and reset it when there is a detection
    //If this counter equals 5 we destroy this tracker.
    int noDetection;

    Point3d position;
    Mat getBvtHistogram();

    Point3d positionHistory[100];
    cv::Rect_<int> rectHistory[5];
    cv::Rect rect;

    PersonModel(Point3d detectedPosition, cv::Rect_<int> bb, int id, int median_window, Mat bvtHistogram);
    Point3d medianFilter();
    ~PersonModel();
    Point3d getPositionEstimate();
    void updateModel();
    Point3d getNearestPoint(vector<cv::Point3d> coordsInBaseFrame, Point3d estimation);
    Point2d velocityMedianFilter();
};

class PersonList
{

public:

    int nPersons;
    int median_window;
    int numberOfFramesBeforeDestruction;
    int numberOfFramesBeforeDestructionLocked;
    double associatingDistance;
    void associateData(vector<cv::Point3d> coordsInBaseFrame, vector<cv::Rect_<int> > rects, vector<Mat> colorFeaturesList);
    void addPerson(Point3d pos, cv::Rect_<int> rect, Mat bvtHistogram);
    std::vector<PersonModel> personList;
    void updateList();

    PersonList(int median_window, int numberOfFramesBeforeDestruction, int numberOfFramesBeforeDestructionLocked, double associatingDistance);

    //Returns a vector containing positions associated to each tracker, that are valid after the median filter
    std::vector<PersonModel> getValidTrackerPosition();


};

#endif // PERSONMOTIONMODEL_HPP
