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


//#define FRONT_WALL_SEARCH_BACKUP_TIME_SLOT 150
//#define FRONT_WALL_SEARCH_ROTATION_TIME_SLOT 150

#define FRONT_WALL_SEARCH_BACKUP_TIME_SLOT 75
#define FRONT_WALL_SEARCH_ROTATION_TIME_SLOT 75

//#define RIGHT_WALL_SEARCH_FORWARD_TIME_SLOT 500
//#define RIGHT_WALL_SEARCH_ROTATION_TIME_SLOT 350
//#define RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT -15

#define RIGHT_WALL_SEARCH_FORWARD_TIME_SLOT 250
#define RIGHT_WALL_SEARCH_ROTATION_TIME_SLOT 175
#define RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT -7

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

#define ALIGNMENT_THRESHOLD 0.7
#define OUTOF_CONTROL_THRESHOLD 0.4
#define LOWER_BOUND_OF_VALID_THRESHOLD 0.3

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
    NS_SURVEY,
    NS_POST_SURVEY_ALIGN,
    NS_FOLLOW_WALL,
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

    int sleepMS = 10000;
    //robot.sendDriveCommand (200, Create::DRIVE_STRAIGHT);
    //robot.sendDriveCommand (200, Create::DRIVE_INPLACE_CLOCKWISE);
    robot.sendDriveCommand (100, -200);
    cout<<"start sleep"<<endl;
    this_thread::sleep_for(chrono::milliseconds(sleepMS));
    cout<<"end sleep"<<endl;
    robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);



    return;
#endif


    const int SEARCHING_SPEED = 100;
    const int MID_BACKUP_DIST_mm = 50;
    const int NS_SURVEY_SLOT_MAX = 1000; // set it for a 360 degree
    const int CLOCK_WISE_RADIOUS = -10;
    const int ANTICLOCK_WISE_RADIOUS = 10;
    const int SEARCH_RIGHT_WALL_RADIOUS = -200;


    const int FOLLOW_WALL_CHECK_SIGNAL_INTERVAL = 8;


    int ns_survey_slotCount = 0;


    NAVIGATION_STATUS navigationStatus;
    int consecutiveOperation = 0;
    int backupTimeSlot = 0;
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
                                ns_survey_slotCount = 0;
                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                                surveyManagerPtr->finalizeSurvey();
                                navigationStatus = NS_POST_SURVEY_ALIGN;

                            }else {
                                robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                            }
                        }
                        else
                        {// search for atleast 360 degree... if no wall found, go to SEARCH state
                            ns_survey_slotCount = 0;
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                            navigationStatus = NS_SEARCHING;
                        }

                        break;
                    }
                    case NS_POST_SURVEY_ALIGN:
                    {

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
                    case NS_SEARCH_RIGHT_WALL:
                    {//rotationTimeSlot = some value;
                        //backupTimeSlot = some value;

                        if(0 < backupTimeSlot)
                        {
                            cout<<"\tbackupTimeSlot = "<<backupTimeSlot<<endl;
                            backupTimeSlot--;

                            robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);

                            if(0 == backupTimeSlot)
                                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);

                            break;
                        }
                        else if(0 < rotationTimeSlot)
                        {
                            cout << "\trotationTimeSlot = "<< rotationTimeSlot<<endl;
                            rotationTimeSlot--;

                            robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

                            if(0 == rotationTimeSlot)
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            break;
                        }
                        else
                        {

                            cout <<"\t else of NS_SEARCH_RIGHT_WALL"<<endl;
                            if(robot.bumpLeft())
                            {
                                cout <<"\t\t\t TODO: handle corner case... it should not be a problem for mp2... inside NS_SEARCH_RIGHT_WALL"<<endl;
                                //TODO: search for front wall
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                            }
                            else if(robot.bumpRight())
                            {
                                if(wallSigMgr.isNoWallSignal())
                                {
                                    cout << "\tinside NS_SEARCH_RIGHT_WALL, GO TO -> NS_SURVEY"<<endl;
                                    backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

                                    navigationStatus = NS_SURVEY;
                                    NS_SURVEY_ISwallAvgHighValueSeen = false;
                                    ns_survey_slotCount = 0;

                                }
                                else
                                {
                                    cout << "\tinside NS_SEARCH_RIGHT_WALL, GO TO -> NS_PRE_SURVEY"<<endl;
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


                        break;
                    }
                    case NS_FOLLOW_WALL:
                    {// REQUIREMENT: consecutiveOperation = 0

                        if(robot.bumpLeft())
                        {

                            //TODO: search for front wall
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                            /////////cout<<"BUMP LEFT :: NS_FOLLOW_WALL  - >  NS_SEARCH_FRONT_WALL"<<endl;
                            /////////////////////////////////////////////////////////////navigationStatus = NS_SEARCH_FRONT_WALL;
                            ////backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
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
                                navigationStatus = NS_SEARCH_RIGHT_WALL;
                            }
                            else
                            {// if there is wall signal, do the pre-survay thing
                                //TODO: adjust the threshold
                                cout << "\t WALL signal found....  GO TO -> NS_PRE_SURVEY"<<endl;

                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                navigationStatus = NS_PRE_SURVEY;
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
                                        navigationStatus = NS_SEARCH_RIGHT_WALL;
                                    }
                                    else
                                    {
                                        if( (LOWER_BOUND_OF_VALID_THRESHOLD <=surveyManagerPtr->getSignalStrength(wallSignal) ) && (surveyManagerPtr->getSignalStrength(wallSignal) < OUTOF_CONTROL_THRESHOLD))
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



                        /*
                        if(0 < backupTimeSlot)
                        {
                            backupTimeSlot--;

                            robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);

                            if(0 == backupTimeSlot) {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                rotationTimeSlot = NS_FOLLOW_WALL_LEFT_BUMP_ROTATION_TIME_SLOT;
                            }

                            break;
                        }
                        else if((0 == backupTimeSlot) && (0 < rotationTimeSlot) )
                        {

                            robot.sendDriveCommand(ALIGNMENT_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                            rotationTimeSlot--;
                            if(0 == rotationTimeSlot)
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                            }
                        }
                        else
                        {

                            if(robot.bumpLeft())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                cout<<"BUMP LEFT :: NS_FOLLOW_WALL  - >  NS_SEARCH_FRONT_WALL"<<endl;
                                navigationStatus = NS_SEARCH_FRONT_WALL;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                            }
                            else if(robot.bumpRight())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                cout<<"BUMP RIGHT :: NS_FOLLOW_WALL  - >  rotate counter clockwise"<<endl;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                            }
                            else
                            {
                                if(wallSigMgr.isNoWallSignal())
                                {
                                    cout<<"NO WALL SIGNAL :: Search for right wall"<<endl;
                                    rotationTimeSlot = RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT;
                                    navigationStatus = NS_SEARCH_RIGHT_WALL;
                                    backupTimeSlot = RIGHT_WALL_SEARCH_FORWARD_TIME_SLOT;  // shegufta: instead of declearing a new variable for forwardTimeSlot, to keep thing simple, I have just used backupTimeSlot

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
                                        if(consecutiveOperation < 4)
                                        {
                                            robot.sendDriveCommand(FOLLOW_WALL_SPEED, Create::DRIVE_STRAIGHT);
                                            consecutiveOperation++;
                                        }
                                        else
                                        {
                                            consecutiveOperation=0;
                                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                                            if( (LOWER_BOUND_OF_VALID_THRESHOLD <=surveyManagerPtr->getSignalStrength(wallSignal) ) && (surveyManagerPtr->getSignalStrength(wallSignal) < OUTOF_CONTROL_THRESHOLD))
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

                        }
                        */

                        break;
                    }
                    case NS_SEARCH_FRONT_WALL:
                    {
                        if(0 < backupTimeSlot)
                        {
                            robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            backupTimeSlot--;

                            if(0 == backupTimeSlot)
                            {
                                cout << " inside NS_SEARCH_FRONT_WALL,  backoff end, now it will start rotating counter clockwise"<<endl;
                                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                                rotationTimeSlot = FRONT_WALL_SEARCH_ROTATION_TIME_SLOT;
                            }
                            break;
                        }
                        else if(0 < rotationTimeSlot)
                        {
                            robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

                            rotationTimeSlot--;

                            if(0 == rotationTimeSlot)
                            {
                                cout << " inside NS_SEARCH_FRONT_WALL,  counterclockwise rotation end...."<<endl;
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                            }
                            break;
                        }


                        if(robot.bumpLeft() )
                        {
                            cout << " inside NS_SEARCH_FRONT_WALL,  BUMP LEFT"<<endl;
                            backupTimeSlot = MID_BACKUP_TIME_SLOT;
                            break;
                        }
                        else if ( robot.bumpRight() )
                        {
                            cout<<"inside NS_SEARCH_FRONT_WALL -> BUMP RIGHT : next State ->";
                            if(wallSigMgr.isNoWallSignal())
                            {
                                cout << "NS_SURVEY"<<endl;
                                backupTimeSlot = SHORT_BACKUP_TIME_SLOT;

                                navigationStatus = NS_SURVEY;
                                NS_SURVEY_ISwallAvgHighValueSeen = false;

                            }
                            else
                            {
                                cout << "NS_PRE_SURVEY"<<endl;
                                backupTimeSlot = MID_BACKUP_TIME_SLOT;
                                navigationStatus = NS_PRE_SURVEY;
                            }

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            break;

                        }

                        robot.sendDriveCommand (SEARCHING_SPEED, Create::DRIVE_STRAIGHT);

                        break;
                    }

                 /*
                    case NS_SEARCH_RIGHT_WALL:
                    {
                        if(rotationTimeSlot < 0)
                        {
                            robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                            rotationTimeSlot++;


                            if(0 == rotationTimeSlot)
                            {
                                cout << " inside NS_SEARCH_RIGHT_WALL,  COUNTER-clockwise rotation end...."<<endl;
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                            }
                            break;
                        }
                        else if(0 < backupTimeSlot) // NOTE: here backupTimeSlot is used to move forward. I have not declared another new variable !
                        {
                            if(robot.bumpLeft() || robot.bumpRight())
                            {
                                backupTimeSlot = MID_BACKUP_TIME_SLOT;
                                navigationStatus = NS_SEARCHING;
                                break;
                            }


                            robot.sendDriveCommand (SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            backupTimeSlot--;

                            cout <<"NS_SEARCH_RIGHT_WALL :: backupTimeSlot = "<<backupTimeSlot <<endl;

                            if(0 == backupTimeSlot)
                            {
                                cout << " inside NS_SEARCH_RIGHT_WALL,  forwording end, now it will start rotating clockwise"<<endl;
                                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                                rotationTimeSlot = RIGHT_WALL_SEARCH_ROTATION_TIME_SLOT;
                            }

                            break;
                        }
                        else if(0 < rotationTimeSlot)
                        {
                            robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);

                            rotationTimeSlot--;

                            if(0 == rotationTimeSlot)
                            {
                                cout << " inside NS_SEARCH_RIGHT_WALL,  clockwise rotation end...."<<endl;
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                            }

                            break;
                        }

                        if(robot.bumpLeft() )
                        {
                            cout << " inside NS_SEARCH_RIGHT_WALL,  BUMP LEFT"<<endl;
                            navigationStatus = NS_SEARCHING;
                            backupTimeSlot = MID_BACKUP_TIME_SLOT;
                            rotationTimeSlot = 0;
                            break;
                        }
                        else if(robot.bumpRight())
                        {
                            cout<<"inside NS_SEARCH_RIGHT_WALL -> BUMP RIGHT : next State ->";
                            if(wallSigMgr.isNoWallSignal())
                            {
                                cout << "NS_SURVEY"<<endl;
                                backupTimeSlot = SHORT_BACKUP_TIME_SLOT;

                                navigationStatus = NS_SURVEY;
                                NS_SURVEY_ISwallAvgHighValueSeen = false;

                            }
                            else
                            {
                                cout << "NS_PRE_SURVEY"<<endl;
                                backupTimeSlot = MID_BACKUP_TIME_SLOT;
                                navigationStatus = NS_PRE_SURVEY;
                            }

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            break;

                        }




                        robot.sendDriveCommand (SEARCHING_SPEED, Create::DRIVE_STRAIGHT);

                        break;
                    }
                    */

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