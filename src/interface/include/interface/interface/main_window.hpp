/**
 * @file /include/interface/main_window.hpp
 *
 * @brief Qt based gui for interface.
 *
 * @date November 2010
 **/
#ifndef interface_MAIN_WINDOW_H
#define interface_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include <QFileSystemModel>
#include "ui_main_window.h"
#include "roboy_managing_node/myoMaster.hpp"
#include "joint_angle_sensors/jas.h"
#include <tinyxml.h>
#include <fstream>
#include <queue>

#define RUN_IN_THREAD
#define NUMBER_OF_FPGAS 5
#define NUMBER_OF_MOTORS_PER_FPGA 14
#define JOINT_ANGLE_SAMPLE_RATE  0.004
#define DISPLACEMENT_LOG_SIZE 10
#define JOINT_NUMBER = 4
#define DERIVATIVE_WINDOW = 20;
#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace interface {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    MyoMaster *myoMaster;
private:
    void MotorStatus(const communication::MotorStatus::ConstPtr &msg);
    void MotorRecordPack(const communication::MotorRecord::ConstPtr &msg);
    void updateSetPointsFromControl( const joint_angle_sensors::jas::ConstPtr &msg );
public Q_SLOTS:
	void on_actionAbout_triggered();
    void updateSetPoints(int percent);
    void updateSetPointsAll(int percent);
	void updateControllerParams();
    void movementPathChanged();
    void recordMovement();
    void plotData(int id);
    bool playMovement();
    void stopMovement();
    void rewindMovement();
    void pauseMovement();
    void loopMovement();
    void stopButtonClicked();
Q_SIGNALS:
    void newData(int id);
private:
    double top1angle(double raw_value);
	Ui::MainWindowDesign ui;
    ros::NodeHandlePtr nh;
    ros::Publisher motorConfig, motorRecordConfig, motorTrajectory, motorTrajectoryControl;
    ros::Subscriber motorStatus, motorRecord, motorAngleReadings;
    QVector<double> time;
    QVector<double> motorData[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA][4];
    long int counter = 0;
    int samples_per_plot = 300;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
	QColor color_pallette[14] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
							   Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray};
    QFileSystemModel *model;
    int numberOfRecordsToWaitFor = 0;
    map<int, vector<int32_t>[NUMBER_OF_MOTORS_PER_FPGA]> records;

	// Control Variables
	double desiredPosition[ 4 ] = { -12     // Joint 1 lower left
                                , -24   // Joint 2 upper left
                                , 32    // Joint 3 upper right
                                , 146 };//Joint 4 upper rigt
	double error[ 4 ] = {0,0,0,0};
    double angleData[4] = {0,0,0,0};
	double pastError[ 4 ] = {0,0,0,0};
	long accumulatedError[ 4 ] =  {0,0,0,0};
	double controlGain[4] = { 16    // Joint 1 lower left
                        , 20 // Joint 2 upper left
                        , 20 // Joint 3 upper right
                        , 16 };//Joint 4 upper rigt
	double controlDerivativeGain[4] = {0.01
										,0.4
										,0.4
										,0.01};
	double controlIntegralGain[4] = {0.0001
									,0.0008
									,0.0008
									,0.0001};
    int rotorForceDirection[NUMBER_OF_MOTORS_PER_FPGA];

};



}  // namespace interface

#endif // interface_MAIN_WINDOW_H
