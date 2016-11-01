#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <climits>
#include <list>
#include <algorithm>
//#include <raspicam/raspicam_cv.h>
//#include <opencv2/imgproc/imgproc.hpp>


#define PRINT_NAV_STATUS

using namespace iRobot;
using namespace LibSerial;
using namespace std;


//if wall sensor's value is lower than this value, it will be considered as zero
#define WALL_SENSOR_MIN 4

#define CLIFF_SENSOR_THRESHOLD 10


#define MID_BACKUP_TIME_SLOT 50
#define SHORT_BACKUP_TIME_SLOT 50


//#define FRONT_WALL_SEARCH_BACKUP_TIME_SLOT 75
//#define FRONT_WALL_SEARCH_ROTATION_TIME_SLOT 75

//#define RIGHT_WALL_SEARCH_FORWARD_TIME_SLOT 500
//#define RIGHT_WALL_SEARCH_ROTATION_TIME_SLOT 350
//#define RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT -15

//#define RIGHT_WALL_SEARCH_FORWARD_TIME_SLOT 250
//#define RIGHT_WALL_SEARCH_ROTATION_TIME_SLOT 175
//#define RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT -7

//#define NS_FOLLOW_WALL_LEFT_BUMP_ROTATION_TIME_SLOT 10
#define NS_FOLLOW_WALL_LEFT_BUMP_ROTATION_TIME_SLOT 5
#define SKIP_DUE_TO_OVERCURRENT_SLOT 0

//#define SEARCHING_SPEED 50
//#define FOLLOW_WALL_SPEED 50
//#define SURVEY_SPEED 50
//#define ALIGNMENT_SPEED 50

//#define SEARCHING_SPEED 100
#define FOLLOW_WALL_SPEED 100
#define SURVEY_SPEED 100
#define ALIGNMENT_SPEED 100

//#define ALIGNMENT_THRESHOLD 0.7
//#define OUTOF_CONTROL_THRESHOLD 0.4
//#define LOWER_BOUND_OF_VALID_THRESHOLD 0.3

class WallSignalManager
{
private:
    int WALL_SIGNAL_HISTORY_SIZE;
    short* wallSignalHistoryArray = NULL;

public:

    WallSignalManager(int _wallSigHistorySize)
    {
        WALL_SIGNAL_HISTORY_SIZE = _wallSigHistorySize;

        wallSignalHistoryArray = new short[WALL_SIGNAL_HISTORY_SIZE];

        for(int i=0 ; i<WALL_SIGNAL_HISTORY_SIZE ; i++)
            wallSignalHistoryArray[i] = 0;
    }

    ~WallSignalManager()
    {
        delete[] wallSignalHistoryArray;
        wallSignalHistoryArray = NULL;
    }

    void appendHistory(short currentWallSignal)
    {
        int index = 1;
        for(; index< WALL_SIGNAL_HISTORY_SIZE; index++)
        {
            wallSignalHistoryArray[index-1] = wallSignalHistoryArray[index];
        }
        wallSignalHistoryArray[index-1] = currentWallSignal;
    }

    short getAverage()
    {
        double sum = 0.0;

        for(int index = 0; index< WALL_SIGNAL_HISTORY_SIZE; index++)
        {
            sum += wallSignalHistoryArray[index];
        }

        return (short)(sum / (double)WALL_SIGNAL_HISTORY_SIZE);
    }

    bool isIncreasing()
    {
        int del = wallSignalHistoryArray[WALL_SIGNAL_HISTORY_SIZE-1] - wallSignalHistoryArray[0];

        return (0<=del);
    }

    bool isNoWallSignal()
    {
        return (getAverage() < WALL_SENSOR_MIN);
    }
};

class SurveyManager
{
public:
    std::list<short> wallSensorList;
    std::list<short>::iterator listIT;
    bool isFinalized = false;

    short min;
    short max;

    void init()
    {
        wallSensorList.clear();
    }

    SurveyManager()
    {
        init();
    }

    void pushSurveyData(short wallSensor)
    {
        if(false == isFinalized)
            wallSensorList.push_back(wallSensor);
    }

    void finalizeSurvey()
    {
        wallSensorList.sort();

        min = *(std::min_element(wallSensorList.begin(), wallSensorList.end()));
        max = *(std::max_element(wallSensorList.begin(), wallSensorList.end()));

        isFinalized = true;
    }

    double getSignalStrength(short wallSensor)
    {
        if(isFinalized)
        {
            if(wallSensor < min)
                return 0.0;
            else if(max < wallSensor)
                return 1.0;
            else
            {
                double index = 0.0;

                for(listIT = wallSensorList.begin() ; listIT != wallSensorList.end()  ; ++listIT)
                {
                    if(*listIT <= wallSensor)
                        index++;
                    else
                        break;

                }

                return (index/(double)wallSensorList.size());
            }

        }

        return -1.0;

    }



};

enum NAVIGATION_STATUS
{
    NS_SEARCHING,
    NS_PRE_SURVEY,
    NS_ALIGN_FOR_PRE_SURVEY, //it will be used only for the transition from NS_FOLLOW_WALL to NS_PRE_SURVEY... If robot is inside FOLLOW_WALL , bumpRight()==true and isWallSignal()==true, that means robot is to close to wall... so take a left-back, then hit forward to the wall
    NS_SURVEY,
    NS_POST_SURVEY_ALIGN,
    NS_FOLLOW_WALL,
    NS_ROTATE_RIGHT_AND_SEARCH,
    NS_SEARCH_FRONT_WALL,
    NS_SEARCH_RIGHT_WALL
};


void navigate(void*);


///////////////////////////////////////////////////////////////////////////////

int calculateTimeSlot(double slotDuration_mSec, double velocity_mmPerSec, double distance_mm)
{
    int slot;

    double time_mSec = (distance_mm/velocity_mmPerSec)*1000.0;

    slot = ceil( time_mSec / slotDuration_mSec );

    return slot;
}

void navigate(void* _robot)
{
    Create robot = *((Create*) _robot);




#if false

    const int temp_SEARCHING_SPEED = 100;
    int temp_sleepTimeMS = 15;

    //int sleepMS = 10000;
    //robot.sendDriveCommand (200, Create::DRIVE_STRAIGHT);
    //robot.sendDriveCommand (200, Create::DRIVE_INPLACE_CLOCKWISE);
    //robot.sendDriveCommand (100, -200);
    //cout<<"start sleep"<<endl;
    //this_thread::sleep_for(chrono::milliseconds(sleepMS));
    //cout<<"end sleep"<<endl;
    //robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);

    int counter = 0;
    while (!robot.playButton ())
    {
        counter++;
        cout << "counter = "<<counter<<endl;
        robot.sendDriveCommand(temp_SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
        this_thread::sleep_for(chrono::milliseconds(temp_sleepTimeMS));
    }

    cout << "Play button pressed, stopping Robot" << endl;
    robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);



    return;
#endif

    const double ALIGNMENT_THRESHOLD = 0.7;
    const double LOWER_BOUND_OF_VALID_THRESHOLD = 0.3;

    const double INIT_OUTOF_CONTROL_THRESHOLD = 0.4;
    double OUTOF_CONTROL_THRESHOLD = INIT_OUTOF_CONTROL_THRESHOLD;


    const int RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT = 7;

    const int SEARCHING_SPEED = 100;
    const int MID_BACKUP_DIST_mm = 40;
    const int SEARCH_R_WALL_ForwardDist_mm = 120;
    const int NS_SURVEY_SLOT_MAX = 600; // if slot time is 15ms and speed is 100mmps, then 580 will rotate the robot 360 degree | we have set it to 600... if the problem is not solved in 600 slot, then there is something wrong... hence retry
    const int CLOCK_WISE_RADIOUS = -10;
    const int ANTICLOCK_WISE_RADIOUS = 10;
    const int SEARCH_RIGHT_WALL_RADIOUS = -185;  // radious 185 is ok when the searching speed is 100mmps

    const int REVERSE_ROTATION_RADIOUS = 100;
    const int REVERSE_ROTATION_TIME_SLOT = 150;

    const int SEARCH_F_WALL_BACKUP_DIST_mm = 80;
    const int SEARCH_F_WALL_RADIOUS = 50;
    const int FRONT_WALL_SEARCH_ROTATION_TIME_SLOT = 100;


    const int FOLLOW_WALL_CHECK_SIGNAL_INTERVAL = 8;


    int ns_survey_slotCount = 0;


    NAVIGATION_STATUS navigationStatus;
    int consecutiveOperation = 0;
    int backupTimeSlot = 0;
    int forwardTimeSlot = 0;
    int rotationTimeSlot = 0;
    bool NS_SURVEY_ISwallAvgHighValueSeen = false;
    int alignLeft = 0;
    int alignRight = 0;
    int skipForOvercurrent = 0;

    int sleepTimeMS = 15;


    SurveyManager *surveyManagerPtr = NULL;
    WallSignalManager wallSigMgr(FOLLOW_WALL_CHECK_SIGNAL_INTERVAL);

    navigationStatus = NS_SEARCHING;
    surveyManagerPtr = NULL;


    try
    {

        while (!robot.playButton ())
        {
            short wallSignal = robot.wallSignal();
            wallSigMgr.appendHistory(wallSignal);

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /*
            bool wheeldropCaster =  robot.wheeldropCaster();
            bool wheeldropLeft = robot.wheeldropLeft();
            bool wheeldropRight = robot.wheeldropRight();

            bool leftWheelOvercurrent = robot.leftWheelOvercurrent();
            bool rightWheelOvercurrent = robot.rightWheelOvercurrent();


            short cliffLeftSignal = robot.cliffLeftSignal();
            short cliffRightSignal = robot.cliffRightSignal();
            short cliffFrontLeftSignal = robot.cliffFrontLeftSignal();
            short cliffFrontRightSignal = robot.cliffFrontRightSignal();

            bool IsCliffSensorsOK = ((CLIFF_SENSOR_THRESHOLD < cliffLeftSignal) && (CLIFF_SENSOR_THRESHOLD<cliffRightSignal) && (CLIFF_SENSOR_THRESHOLD<cliffFrontLeftSignal)&& (CLIFF_SENSOR_THRESHOLD<cliffFrontRightSignal));
            bool IsWheelDropOk = !(wheeldropCaster || wheeldropLeft || wheeldropRight);
            bool IsOvercurrentOK = !(leftWheelOvercurrent || rightWheelOvercurrent);
             */
            bool IsCliffSensorsOK = true;
            bool IsWheelDropOk = true;
            bool IsOvercurrentOK = true;
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if(!(IsCliffSensorsOK && IsWheelDropOk && IsOvercurrentOK))
            {// check sensors... drop sensor etc
                cout <<"IsCliffSensorsOK = "<<IsCliffSensorsOK <<" | IsWheelDropOk = "<<IsWheelDropOk <<" | IsOvercurrentOK = "<<IsOvercurrentOK<<endl;
                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                if(!IsOvercurrentOK)
                    skipForOvercurrent = SKIP_DUE_TO_OVERCURRENT_SLOT;
            }
            else if(0 < skipForOvercurrent)
            {
                skipForOvercurrent--;
            }
            else
            {

                switch(navigationStatus)
                {
                    case NS_SEARCHING:
                    {
                        if(0 < backupTimeSlot)
                        {
                            backupTimeSlot--;

                            robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);

                            if(0 == backupTimeSlot)
                                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);

                            break;
                        }


                        if(NULL != surveyManagerPtr)
                        {
                            delete surveyManagerPtr;
                            surveyManagerPtr = NULL;
                        }

                        if(robot.bumpLeft() )
                        {
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                            backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

                            cout<<"NS_SEARCHING -> BUMP LEFT : next State ->  NS_SEARCH_FRONT_WALL";
                            navigationStatus = NS_SEARCH_FRONT_WALL;
                        }
                        else if(robot.bumpRight())
                        {
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            cout<<"NS_SEARCHING -> BUMP RIGHT : next State ->";
                            if(wallSigMgr.isNoWallSignal())
                            {
                                cout << "\tGO TO -> NS_SURVEY"<<endl;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

                                navigationStatus = NS_SURVEY;
                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                ns_survey_slotCount = 0;

                            }
                            else
                            {
                                cout << "\tGO TO -> NS_PRE_SURVEY"<<endl;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                navigationStatus = NS_PRE_SURVEY;
                            }



                        }
                        else
                        {
                            robot.sendDriveCommand (SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                        }


                        break;
                    }

                    case NS_PRE_SURVEY:
                    {
                        if(NULL != surveyManagerPtr)
                        {
                            delete surveyManagerPtr;
                            surveyManagerPtr = NULL;
                        }

                        if(0 < backupTimeSlot)
                        {
                            robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            backupTimeSlot--;

                            if(0 == backupTimeSlot)
                                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                        }
                        else
                        {
                            robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);

                            if(wallSigMgr.getAverage()< WALL_SENSOR_MIN)
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                navigationStatus = NS_SURVEY;
                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                ns_survey_slotCount = 0;
                            }
                        }

                        break;
                    }

                    case NS_SURVEY: {// Condition: when it will enter the NS_SURVEY mode FOR THE FIRST TIME, the condition  [ws_getAverage(wallSignal) < WALL_SENSOR_MIN)] will be true
                        //CONDITION FOR THE FIRST TIME : NS_SURVEY_ISwallAvgHighValueSeen = false;
                        //ns_survey_slotCount = 0;
                        if (0 < backupTimeSlot)
                        {
                            robot.sendDriveCommand(-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            backupTimeSlot--;

                            if (0 == backupTimeSlot)
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                        }
                        else if (ns_survey_slotCount < NS_SURVEY_SLOT_MAX)
                        {
                            ns_survey_slotCount++;

                            if (NULL == surveyManagerPtr)
                                surveyManagerPtr = new SurveyManager;

                            surveyManagerPtr->pushSurveyData(wallSignal);

                            if ((!NS_SURVEY_ISwallAvgHighValueSeen) && (!wallSigMgr.isNoWallSignal()))
                                NS_SURVEY_ISwallAvgHighValueSeen = true;


                            if (NS_SURVEY_ISwallAvgHighValueSeen && (wallSigMgr.getAverage() < WALL_SENSOR_MIN))
                            {

                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);


                                surveyManagerPtr->finalizeSurvey();

                                ns_survey_slotCount = 0;
                                navigationStatus = NS_POST_SURVEY_ALIGN;

                            }else {
                                robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                            }
                        }
                        else
                        {

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            if(NULL != surveyManagerPtr)
                            {
                                delete surveyManagerPtr;
                                surveyManagerPtr = NULL;
                            }

                            ns_survey_slotCount = 0;

                            navigationStatus = NS_ROTATE_RIGHT_AND_SEARCH;
                        }

                        break;
                    }
                    case NS_POST_SURVEY_ALIGN:
                    {//condition ns_survey_slotCount = 0;


                        if (ns_survey_slotCount < NS_SURVEY_SLOT_MAX)
                        {
                            ns_survey_slotCount++;


                            //cout <<"\tsurveyManagerPtr->getSignalStrength(wallSignal) = "<<surveyManagerPtr->getSignalStrength(wallSignal)<<"  | ALIGNMENT_THRESHOLD = "<<ALIGNMENT_THRESHOLD<<endl;
                            if (surveyManagerPtr->getSignalStrength(wallSignal) < ALIGNMENT_THRESHOLD)
                                robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
                            else
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                navigationStatus = NS_FOLLOW_WALL;
                                consecutiveOperation = 0;
                                backupTimeSlot = 0;
                                alignLeft = 0;
                                alignRight = 0;

                            }
                            break;

                        }
                        else
                        {

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            if(NULL != surveyManagerPtr)
                            {
                                delete surveyManagerPtr;
                                surveyManagerPtr = NULL;
                            }

                            ns_survey_slotCount = 0;

                            navigationStatus = NS_ROTATE_RIGHT_AND_SEARCH;
                            break;
                        }


                        break;
                    }

                    case NS_ROTATE_RIGHT_AND_SEARCH:
                    {
                        if(NULL != surveyManagerPtr)
                        {
                            delete surveyManagerPtr;
                            surveyManagerPtr = NULL;
                        }

                        cout <<"\t else of NS_ROTATE_RIGHT_AND_SEARCH"<<endl;
                        if(robot.bumpLeft())
                        {
                            cout <<"\t\t\t TODO: handle corner case... it should not be a problem for mp2... inside NS_ROTATE_RIGHT_AND_SEARCH"<<endl;
                            //TODO: search for front wall
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                        }
                        else if(robot.bumpRight())
                        {
                            if(wallSigMgr.isNoWallSignal())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                cout << "\tinside NS_ROTATE_RIGHT_AND_SEARCH, GO TO -> NS_SURVEY"<<endl;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

                                navigationStatus = NS_SURVEY;
                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                ns_survey_slotCount = 0;

                            }
                            else
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                cout << "\tinside NS_ROTATE_RIGHT_AND_SEARCH, GO TO -> NS_PRE_SURVEY"<<endl;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                navigationStatus = NS_PRE_SURVEY;
                            }

                        }
                        else
                        {

                            cout<<"rotate cloclwise"<<endl;

                            robot.sendDriveCommand(SEARCHING_SPEED, SEARCH_RIGHT_WALL_RADIOUS);
                        }
                        break;
                    }
                    case NS_SEARCH_RIGHT_WALL:
                    {//rotationTimeSlot = some value;
                        //backupTimeSlot = some value;

                        cout <<"rotationTimeSlot  = "<<rotationTimeSlot <<endl;

                        if(0 < backupTimeSlot)
                        {
                            cout<<"\tbackupTimeSlot = "<<backupTimeSlot<<endl;
                            backupTimeSlot--;

                            robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);

                            if(0 == backupTimeSlot)
                                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);

                            break;
                        }
                        else if( ((0 < rotationTimeSlot) || (0 < forwardTimeSlot) ) && ((robot.bumpLeft())||(robot.bumpRight())) )
                        {
                            if(robot.bumpLeft())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                cout <<"\t\t\t TODO: handle corner case... it should not be a problem for mp2... inside NS_SEARCH_RIGHT_WALL"<<endl;
                                //TODO: search for front wall

                            }
                            else if(robot.bumpRight())
                            {
                                if(wallSigMgr.isNoWallSignal())
                                {
                                    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                    cout << "\tinside NS_SEARCH_RIGHT_WALL, else_if GO TO -> NS_SURVEY"<<endl;
                                    backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

                                    navigationStatus = NS_SURVEY;
                                    NS_SURVEY_ISwallAvgHighValueSeen = false;
                                    ns_survey_slotCount = 0;

                                }
                                else
                                {
                                    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                    cout << "\tinside NS_SEARCH_RIGHT_WALL, else_if GO TO -> NS_PRE_SURVEY"<<endl;
                                    backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                    navigationStatus = NS_PRE_SURVEY;
                                }

                            }

                            break;

                        }
                        else if(0 < rotationTimeSlot)
                        {
                            //cout << "\trotationTimeSlot = "<< rotationTimeSlot<<endl;
                            rotationTimeSlot--;

                            robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

                            if(0 == rotationTimeSlot)
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            break;
                        }
                        else if(0 < forwardTimeSlot)
                        {
                            forwardTimeSlot--;

                            robot.sendDriveCommand (SEARCHING_SPEED, Create::DRIVE_STRAIGHT);

                            if(0 == forwardTimeSlot)
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            break;
                        }
                        else
                        {
                            navigationStatus = NS_ROTATE_RIGHT_AND_SEARCH;
                            break;
                        }


                        break;
                    }
                    case NS_ALIGN_FOR_PRE_SURVEY:
                    {//rotationTimeSlot and backupTimeSlot
                        if(0 < backupTimeSlot)
                        {
                            backupTimeSlot--;
                            robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);


                            if(0 == backupTimeSlot)
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                rotationTimeSlot = REVERSE_ROTATION_TIME_SLOT;
                            }

                            break;
                        }
                            
                        else if(0 < rotationTimeSlot )
                        {
                            rotationTimeSlot--;

                            robot.sendDriveCommand(-SEARCHING_SPEED, REVERSE_ROTATION_RADIOUS);

                            if(0 == rotationTimeSlot)
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                        }
                        else
                        {
                            if(robot.bumpLeft() || robot.bumpRight())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                navigationStatus = NS_PRE_SURVEY;


                            } else
                            {
                                robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            }

                        }


                        break;
                    }

                    case NS_FOLLOW_WALL:
                    {// REQUIREMENT: consecutiveOperation = 0

                        if(robot.bumpLeft())
                        {

                            //TODO: search for front wall
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                            /////////cout<<"BUMP LEFT :: NS_FOLLOW_WALL  - >  NS_SEARCH_FRONT_WALL"<<endl;
                            backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, SEARCH_F_WALL_BACKUP_DIST_mm );
                            navigationStatus = NS_SEARCH_FRONT_WALL;
                        }
                        else if(robot.bumpRight())
                        {
                            cout<<"BUMP RIGHT inside NS_FOLLOW_WALL"<<endl;

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            if(wallSigMgr.isNoWallSignal())
                            {
                                cout<<"\tNO WALL SIGNAL :: Search for right wall"<<endl;

                                rotationTimeSlot = RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm ); // shegufta: instead of declearing a new variable for forwardTimeSlot, to keep thing simple, I have just used backupTimeSlot
                                forwardTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, SEARCH_R_WALL_ForwardDist_mm );
                                navigationStatus = NS_SEARCH_RIGHT_WALL;
                            }
                            else
                            {// if there is wall signal, do the pre-survay thing
                                //TODO: adjust the threshold
                                cout << "\t WALL signal found....  GO TO -> NS_PRE_SURVEY"<<endl;

                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                rotationTimeSlot = 0;
                                //navigationStatus = NS_PRE_SURVEY;
                                navigationStatus = NS_ALIGN_FOR_PRE_SURVEY;

                            }

                        }
                        else

                        {

                            if(0 < alignLeft)
                            {
                                //robot.sendDriveCommand(ALIGNMENT_SPEED,Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                                robot.sendDriveCommand(SEARCHING_SPEED, ANTICLOCK_WISE_RADIOUS);
                                alignLeft--;
                            }
                            else if(0 < alignRight)
                            {
                                //robot.sendDriveCommand(ALIGNMENT_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
                                robot.sendDriveCommand(SEARCHING_SPEED, CLOCK_WISE_RADIOUS);
                                alignRight--;
                            }
                            else
                            {
                                if(consecutiveOperation < FOLLOW_WALL_CHECK_SIGNAL_INTERVAL)
                                {
                                    robot.sendDriveCommand(FOLLOW_WALL_SPEED, Create::DRIVE_STRAIGHT);
                                    consecutiveOperation++;
                                }
                                else
                                {
                                    consecutiveOperation=0;
                                    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                                    if(wallSigMgr.isNoWallSignal())
                                    {
                                        cout<<"\t NO WALL SIGNAL without bump :: Search for right wall"<<endl;

                                        rotationTimeSlot = 0;
                                        backupTimeSlot = 0; // shegufta: instead of declearing a new variable for forwardTimeSlot, to keep thing simple, I have just used backupTimeSlot
                                        forwardTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, SEARCH_R_WALL_ForwardDist_mm );
                                        navigationStatus = NS_SEARCH_RIGHT_WALL;
                                    }
                                    else
                                    {
                                        if( (LOWER_BOUND_OF_VALID_THRESHOLD <=surveyManagerPtr->getSignalStrength(wallSignal) ) && (surveyManagerPtr->getSignalStrength(wallSignal) < OUTOF_CONTROL_THRESHOLD))
                                        //if( (surveyManagerPtr->getSignalStrength(wallSignal) < OUTOF_CONTROL_THRESHOLD))
                                        {
                                            alignRight = 4;
                                        }
                                        else
                                        {
                                            if (wallSigMgr.isIncreasing())
                                                alignLeft = 1;
                                            else
                                                alignRight = 1;
                                        }

                                    }


                                }

                            }
                        }


                        break;
                    }
                    case NS_SEARCH_FRONT_WALL:
                    {
                        if(0 < backupTimeSlot)
                        {
                            backupTimeSlot--;

                            robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);


                            if(0 == backupTimeSlot)
                            {
                                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                                rotationTimeSlot = FRONT_WALL_SEARCH_ROTATION_TIME_SLOT;
                            }
                            break;
                        }
                        else if(0 < rotationTimeSlot)
                        {
                            rotationTimeSlot--;

                            if(robot.bumpLeft())
                            {
                                //NOTE: this should not be happen... the maze should not be that complex... but if this happens, go to search state
                                cout <<"\n\t\t EXCEPTION: this should not happen ! go to the  NS_SEARCHING state"<<endl;
                                navigationStatus = NS_SEARCHING;
                                break;
                            }
                            else if(robot.bumpRight())
                            {
                                if(wallSigMgr.isNoWallSignal())
                                {
                                    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                    backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

                                    navigationStatus = NS_SURVEY;
                                    NS_SURVEY_ISwallAvgHighValueSeen = false;
                                    ns_survey_slotCount = 0;

                                }
                                else
                                {
                                    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                    backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                    navigationStatus = NS_PRE_SURVEY;
                                }

                            }


                            robot.sendDriveCommand(SEARCHING_SPEED, SEARCH_F_WALL_RADIOUS);

                            //////////////////////////////////////////////////////////

                            if(0 == rotationTimeSlot)
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            break;
                        }
                        else
                        {
                            navigationStatus = NS_ROTATE_RIGHT_AND_SEARCH;
                            break;
                        }

                        break;
                    }

                }

            }


            cout << "Wall signal " << wallSignal << "     sleep time "<<sleepTimeMS << "    navStatus = " << navigationStatus<< endl;

            this_thread::sleep_for(chrono::milliseconds(sleepTimeMS));

        }

        cout << "Play button pressed, stopping Robot" << endl;
        robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);

    }
    //catch(boost::thread_interrupted&)
    catch(exception ex)
    {
    }



}

int main ()
{

    char serial_loc[] = "/dev/ttyUSB0";

    try
    {
        SerialStream stream (serial_loc, LibSerial::SerialStreamBuf::BAUD_57600);
        cout << "Opened Serial Stream to" << serial_loc << endl;
        this_thread::sleep_for(chrono::milliseconds(1000));
        Create robot(stream);
        cout << "Created iRobot Object" << endl;
        robot.sendFullCommand();
        cout << "Setting iRobot to Full Mode" << endl;
        this_thread::sleep_for(chrono::milliseconds(1000));
        cout << "Robot is ready" << endl;

        // Let's stream some sensors.
        Create::sensorPackets_t sensors;
        sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
        sensors.push_back(Create::SENSOR_WALL_SIGNAL);
        sensors.push_back (Create::SENSOR_BUTTONS);

        sensors.push_back (Create::SENSOR_CLIFF_LEFT_SIGNAL);
        sensors.push_back (Create::SENSOR_CLIFF_RIGHT_SIGNAL);
        sensors.push_back (Create::SENSOR_CLIFF_FRONT_LEFT_SIGNAL);
        sensors.push_back (Create::SENSOR_CLIFF_FRONT_RIGHT_SIGNAL );

        sensors.push_back(Create::SENSOR_OVERCURRENTS);


        robot.sendStreamCommand (sensors);
        int ledColor = Create::LED_COLOR_GREEN;
        //robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
        //robot.sendLedCommand (Create::LED_PLAY, 0, 0);
        //cout << "Sent Drive Command" << endl;
        cout << "Sent Stream Command" << endl;
        // Let's turn!

        navigate((void*)&robot);

    }
    catch (InvalidArgument& e)
    {
        cerr << e.what () << endl;
        return 3;
    }
    catch (CommandNotAvailable& e)
    {
        cerr << e.what () << endl;
        return 4;
    }
}