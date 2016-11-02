#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <climits>
#include <list>
#include <map>
#include <algorithm>
//#include <raspicam/raspicam_cv.h>
//#include <opencv2/imgproc/imgproc.hpp>


#define PRINT_NAV_STATUS

using namespace iRobot;
using namespace LibSerial;
using namespace std;


//if wall sensor's value is lower than this value, it will be considered as zero




void navigate(void*);

class WallSignalManager
{
private:
    int WALL_SIGNAL_HISTORY_SIZE;
    int WALL_SENSOR_MIN;
    short* wallSignalHistoryArray = NULL;

public:

    WallSignalManager(int _wallSigHistorySize, int _WALL_SENSOR_MIN)
    {
        WALL_SIGNAL_HISTORY_SIZE = _wallSigHistorySize;
        WALL_SENSOR_MIN = _WALL_SENSOR_MIN;

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
    NS_MOVE_AWAY_FROM_WALL, //it will be used only for the transition from NS_FOLLOW_WALL to NS_PRE_SURVEY... If robot is inside FOLLOW_WALL , bumpRight()==true and isWallSignal()==true, that means robot is to close to wall... so take a left-back, then hit forward to the wall
    NS_SURVEY,
    NS_POST_SURVEY_ALIGN,
    NS_FOLLOW_WALL,
    NS_ROTATE_RIGHT_AND_SEARCH,
    NS_SEARCH_FRONT_WALL,
    NS_SEARCH_RIGHT_WALL,
    NS_PROBE_RIGHT_WALL
};

std::map<NAVIGATION_STATUS,string> g_stateNameMap;



///////////////////////////////////////////////////////////////////////////////////
class PositionTrackerTuple
{

public:
    NAVIGATION_STATUS navStatus;
    int timeSlotSpent;
    PositionTrackerTuple(NAVIGATION_STATUS _navStatus, int _timeSlotSpent)
    {
        navStatus = _navStatus;
        timeSlotSpent = _timeSlotSpent;
    }
};

std::list<PositionTrackerTuple> g_positionTrackerList;

void g_AddPosition_RESET_current_state_slotCount(NAVIGATION_STATUS _navStatus, int & _timeSlotSpent)
{
    g_positionTrackerList.push_back(PositionTrackerTuple(_navStatus, _timeSlotSpent));
    _timeSlotSpent = 0;
}

void g_printPositionLog()
{
    std::list<PositionTrackerTuple>::iterator it;


    for (it=g_positionTrackerList.begin(); it != g_positionTrackerList.end(); ++it)
    {
        cout<<"state : "<<g_stateNameMap[(*it).navStatus] <<"  | timeSlot = "<<(*it).timeSlotSpent<<endl;
    }

}

//////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////

int calculateTimeSlot(double slotDuration_mSec, double velocity_mmPerSec, double distance_mm)
{
    int slot;

    double time_mSec = (distance_mm/velocity_mmPerSec)*1000.0;

    slot = ceil( time_mSec / slotDuration_mSec );

    return slot;
}




bool g_Is_Taking_Picture = false;
NAVIGATION_STATUS g_navigationStatus;


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


    const int RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT = 30;

    const int SEARCHING_SPEED = 100;
    const int MID_BACKUP_DIST_mm = 40;
    const int SEARCH_R_WALL_ForwardDist_mm = 120;
    const int NS_SURVEY_SLOT_MAX = 600; // if slot time is 15ms and speed is 100mmps, then 580 will rotate the robot 360 degree | we have set it to 600... if the problem is not solved in 600 slot, then there is something wrong... hence retry
    const int CLOCK_WISE_RADIOUS = -10;
    const int ANTICLOCK_WISE_RADIOUS = 10;
    const int SEARCH_RIGHT_WALL_RADIOUS = -185;  // radious 185 is ok when the searching speed is 100mmps



    const int SEARCH_PROBE_WALL_RADIOUS = -230;

    const int REVERSE_ROTATION_RADIOUS = 100;
    const int REVERSE_ROTATION_TIME_SLOT = 75;

    const int SEARCH_F_WALL_BACKUP_DIST_mm = 80;
    const int SEARCH_F_WALL_RADIOUS = 50;
    const int FRONT_WALL_SEARCH_ROTATION_TIME_SLOT = 100;

    const int FOLLOW_WALL_SPEED = 100;

    const int CLIFF_SENSOR_THRESHOLD = 10;
    const int WALL_SENSOR_MIN = 4;


    const int FOLLOW_WALL_CHECK_SIGNAL_INTERVAL = 8;

    //////////////////////////
    const int SKIP_DUE_TO_OVERCURRENT_SLOT = 6;
    const int MAX_OVERCURRENT_SAFE_SLOT = 3;
    int overcurrentSlotCounter = 0;
    ////////////////////////////


    int current_state_slotCount = 0;
    int rotationLimiter = 0;
    int goBackPreviousPosition = 0;


    int consecutiveOperation = 0;
    int backupTimeSlot = 0;
    int forwardTimeSlot = 0;
    int rotationTimeSlot = 0;
    bool NS_SURVEY_ISwallAvgHighValueSeen = false;
    int alignLeft = 0;
    int alignRight = 0;
    int skipForOvercurrent = 0;

    int sleepTimeMS = 15;

    bool isProbeRightWall_SecondTest = false;


    SurveyManager *surveyManagerPtr = NULL;
    WallSignalManager wallSigMgr(FOLLOW_WALL_CHECK_SIGNAL_INTERVAL, WALL_SENSOR_MIN);

    g_navigationStatus = NS_SEARCHING;
    surveyManagerPtr = NULL;


    g_Is_Taking_Picture = false;
    try
    {

        while (!robot.playButton ())
        {
            short wallSignal = robot.wallSignal();
            wallSigMgr.appendHistory(wallSignal);

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////


            if(g_Is_Taking_Picture)
            {
                // TODO: stop for taking picture
                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
            }
            else if(!(IsCliffSensorsOK && IsWheelDropOk && IsOvercurrentOK))
            {// check sensors... drop sensor etc

                if(!(IsCliffSensorsOK && IsWheelDropOk))
                {//if atleast one of them is creating problem, shutdown immediately
                    cout <<"IsCliffSensorsOK = "<<IsCliffSensorsOK <<" | IsWheelDropOk = "<<endl;
                    robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                    // TODO: turn on alarm
                }


                if(!IsOvercurrentOK)
                {
                    ++overcurrentSlotCounter;

                    if(MAX_OVERCURRENT_SAFE_SLOT < overcurrentSlotCounter)
                    {
                        cout <<" | IsOvercurrentOK = "<<IsOvercurrentOK<< " | overcurrentSlotCounter= "<<overcurrentSlotCounter<<endl;
                        // TODO: turn on alarm
                        robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                        skipForOvercurrent = SKIP_DUE_TO_OVERCURRENT_SLOT;
                        overcurrentSlotCounter = 0;
                    }
                }

            }
            else if(0 < skipForOvercurrent)
            {
                skipForOvercurrent--;
            }
            else
            {
                // TODO: turn OFF alarm

                overcurrentSlotCounter = 0;

                switch(g_navigationStatus)
                {
                    case NS_SEARCHING:
                    {
                        current_state_slotCount++;

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

                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

                            cout<<"NS_SEARCHING -> BUMP LEFT : next State ->  NS_SEARCH_FRONT_WALL";

                            g_navigationStatus = NS_SEARCH_FRONT_WALL;
                        }
                        else if(robot.bumpRight())
                        {
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            cout<<"NS_SEARCHING -> BUMP RIGHT : next State ->";
                            if(wallSigMgr.isNoWallSignal())
                            {
                                cout << "\tGO TO -> NS_SURVEY"<<endl;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );


                                if(NULL != surveyManagerPtr)
                                {
                                    delete surveyManagerPtr;
                                    surveyManagerPtr = NULL;
                                }

                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                rotationLimiter = 0;
                                g_navigationStatus = NS_SURVEY;

                            }
                            else
                            {
                                cout << "\tGO TO -> NS_PRE_SURVEY"<<endl;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                g_navigationStatus = NS_PRE_SURVEY;
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
                        current_state_slotCount++;

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

                            if(wallSigMgr.isNoWallSignal())
                            {
                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                                if(NULL != surveyManagerPtr)
                                {
                                    delete surveyManagerPtr;
                                    surveyManagerPtr = NULL;
                                }

                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                g_navigationStatus = NS_SURVEY;
                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                rotationLimiter = 0;
                            }
                        }

                        break;
                    }

                    case NS_SURVEY: {// Condition: when it will enter the NS_SURVEY mode FOR THE FIRST TIME, the condition  [ws_getAverage(wallSignal) < WALL_SENSOR_MIN)] will be true
                        //CONDITION FOR THE FIRST TIME : NS_SURVEY_ISwallAvgHighValueSeen = false;
                        //rotationLimiter = 0;
                        // Make sure, surveyManagerPtr is a new instance, before moving to this state, free it and set it to null

                        current_state_slotCount++;

                        if (0 < backupTimeSlot)
                        {
                            robot.sendDriveCommand(-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            backupTimeSlot--;

                            if (0 == backupTimeSlot)
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                        }
                        else if (rotationLimiter < NS_SURVEY_SLOT_MAX)
                        {
                            rotationLimiter++; // we will not count the backup time here !

                            if (NULL == surveyManagerPtr)
                                surveyManagerPtr = new SurveyManager;

                            surveyManagerPtr->pushSurveyData(wallSignal);

                            if ((!NS_SURVEY_ISwallAvgHighValueSeen) && (!wallSigMgr.isNoWallSignal()))
                                NS_SURVEY_ISwallAvgHighValueSeen = true;


                            if (NS_SURVEY_ISwallAvgHighValueSeen && (wallSigMgr.isNoWallSignal()))
                            {

                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);


                                surveyManagerPtr->finalizeSurvey();

                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state
                                rotationLimiter = 0;
                                g_navigationStatus = NS_POST_SURVEY_ALIGN;

                            }else {
                                robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                            }
                        }
                        else
                        {

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            g_navigationStatus = NS_ROTATE_RIGHT_AND_SEARCH;
                            rotationLimiter = 0;
                        }

                        break;
                    }
                    case NS_POST_SURVEY_ALIGN:
                    {//condition rotationLimiter = 0;

                        current_state_slotCount++;

                        if (rotationLimiter < NS_SURVEY_SLOT_MAX)
                        {
                            rotationLimiter++;


                            //cout <<"\tsurveyManagerPtr->getSignalStrength(wallSignal) = "<<surveyManagerPtr->getSignalStrength(wallSignal)<<"  | ALIGNMENT_THRESHOLD = "<<ALIGNMENT_THRESHOLD<<endl;
                            if (surveyManagerPtr->getSignalStrength(wallSignal) < ALIGNMENT_THRESHOLD)
                                robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
                            else
                            {
                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                                rotationLimiter=0;
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                g_navigationStatus = NS_FOLLOW_WALL;
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

                            rotationLimiter = 0;

                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            g_navigationStatus = NS_ROTATE_RIGHT_AND_SEARCH;
                            break;
                        }


                        break;
                    }

                    case NS_ROTATE_RIGHT_AND_SEARCH:
                    {
                        current_state_slotCount++;

                        if(NULL != surveyManagerPtr)
                        {
                            delete surveyManagerPtr;
                            surveyManagerPtr = NULL;
                        }

                        if(robot.bumpLeft())
                        {
                            // TODO: handle now ... start searching for front wall
                            cout <<"\t\t\t TODO: handle corner case... it should not be a problem for mp2... inside NS_ROTATE_RIGHT_AND_SEARCH"<<endl;
                            //TODO: search for front wall
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                        }
                        else if(robot.bumpRight())
                        {
                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            if(wallSigMgr.isNoWallSignal())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                cout << "\tinside NS_ROTATE_RIGHT_AND_SEARCH, GO TO -> NS_SURVEY"<<endl;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

                                if(NULL != surveyManagerPtr)
                                {
                                    delete surveyManagerPtr;
                                    surveyManagerPtr = NULL;
                                }

                                g_navigationStatus = NS_SURVEY;
                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                rotationLimiter = 0;

                            }
                            else
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                cout << "\tinside NS_ROTATE_RIGHT_AND_SEARCH, GO TO -> NS_PRE_SURVEY"<<endl;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                g_navigationStatus = NS_PRE_SURVEY;
                            }

                        }
                        else
                        {
                            robot.sendDriveCommand(SEARCHING_SPEED, SEARCH_RIGHT_WALL_RADIOUS);
                        }
                        break;
                    }
                    case NS_SEARCH_RIGHT_WALL:
                    {//rotationTimeSlot = some value;
                        //backupTimeSlot = some value;

                        current_state_slotCount++;

                        if(0 < backupTimeSlot)
                        {
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
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                                if(wallSigMgr.isNoWallSignal())
                                {
                                    backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

                                    if(NULL != surveyManagerPtr)
                                    {
                                        delete surveyManagerPtr;
                                        surveyManagerPtr = NULL;
                                    }

                                    rotationLimiter = 0;
                                    g_navigationStatus = NS_SURVEY;
                                    NS_SURVEY_ISwallAvgHighValueSeen = false;
                                    current_state_slotCount = 0;

                                }
                                else
                                {
                                    cout << "\tinside NS_SEARCH_RIGHT_WALL, else_if GO TO -> NS_PRE_SURVEY"<<endl;
                                    backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                    g_navigationStatus = NS_PRE_SURVEY;
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
                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state
                            g_navigationStatus = NS_ROTATE_RIGHT_AND_SEARCH;
                            break;
                        }

                        break;
                    }
                    case NS_MOVE_AWAY_FROM_WALL:
                    {//rotationTimeSlot and backupTimeSlot
                        current_state_slotCount++;
                            
                        if(0 < rotationTimeSlot )
                        {
                            rotationTimeSlot--;

                            robot.sendDriveCommand(-SEARCHING_SPEED, REVERSE_ROTATION_RADIOUS);

                            if(0 == rotationTimeSlot)
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                backupTimeSlot = 0; // just to be sure !
                            }
                        }
                        else
                        {
                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);

                            if(wallSigMgr.isNoWallSignal())
                            {
                                if(NULL != surveyManagerPtr)
                                {
                                    delete surveyManagerPtr;
                                    surveyManagerPtr = NULL;
                                }

                                backupTimeSlot = 0;//calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm ); // we have already backed up
                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                rotationLimiter = 0;
                                g_navigationStatus = NS_SURVEY;

                            }
                            else
                            {
                                cout << "\tGO TO -> NS_PRE_SURVEY"<<endl;
                                backupTimeSlot = 0;//calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );// we have already backed up
                                g_navigationStatus = NS_PRE_SURVEY;
                            }
                            break;
                        }


                        break;
                    }

                    case NS_FOLLOW_WALL:
                    {// REQUIREMENT: consecutiveOperation = 0
                        current_state_slotCount++;

                        if(robot.bumpLeft())
                        {
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state


                            backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, SEARCH_F_WALL_BACKUP_DIST_mm );
                            g_navigationStatus = NS_SEARCH_FRONT_WALL;
                        }
                        else if(robot.bumpRight())
                        {
                            cout<<"BUMP RIGHT inside NS_FOLLOW_WALL"<<endl;

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            if(wallSigMgr.isNoWallSignal())
                            {
                                cout<<"\n\t ### inside : NS_FOLLOW_WALL NO WALL SIGNAL :: Search for right wall"<<endl;

                                rotationTimeSlot = RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT;
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm ); // shegufta: instead of declearing a new variable for forwardTimeSlot, to keep thing simple, I have just used backupTimeSlot
                                forwardTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, SEARCH_R_WALL_ForwardDist_mm );
                                g_navigationStatus = NS_SEARCH_RIGHT_WALL;
                            }
                            else
                            {// if there is wall signal, do the pre-survay thing
                                //TODO: adjust the threshold

                                rotationTimeSlot = REVERSE_ROTATION_TIME_SLOT;
                                g_navigationStatus = NS_MOVE_AWAY_FROM_WALL;

                            }

                        }
                        else

                        {

                            if(0 < alignLeft)
                            {
                                robot.sendDriveCommand(SEARCHING_SPEED, ANTICLOCK_WISE_RADIOUS);
                                alignLeft--;
                            }
                            else if(0 < alignRight)
                            {
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
                                        g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                                        isProbeRightWall_SecondTest = false;
                                        goBackPreviousPosition = 0;
                                        g_navigationStatus = NS_PROBE_RIGHT_WALL;
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



                    case NS_PROBE_RIGHT_WALL:
                    {
                        current_state_slotCount++;


                        if(isProbeRightWall_SecondTest)
                        {
                            if(0 < backupTimeSlot)
                            {
                                backupTimeSlot--;

                                robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);

                                if(0 == backupTimeSlot)
                                {
                                    robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                                    rotationLimiter = 0;
                                }

                                break;
                            }
                            else if(rotationLimiter < NS_SURVEY_SLOT_MAX/5)
                            {
                                rotationLimiter++;

                                if(!wallSigMgr.isNoWallSignal())
                                {
                                    isProbeRightWall_SecondTest = false;
                                    g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);
                                    g_navigationStatus = NS_PRE_SURVEY;
                                }

                                robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

                                if( NS_SURVEY_SLOT_MAX/5  == rotationLimiter )
                                {
                                    goBackPreviousPosition = rotationLimiter;
                                }
                            }
                            else if(0 < goBackPreviousPosition)
                            {
                                goBackPreviousPosition--;
                                robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
                            }
                            else
                            {
                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);

                                isProbeRightWall_SecondTest = false;

                                rotationTimeSlot = RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT;
                                //backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm ); // shegufta: instead of declearing a new variable for forwardTimeSlot, to keep thing simple, I have just used backupTimeSlot
                                backupTimeSlot = 0; // we have already backed up
                                forwardTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, SEARCH_R_WALL_ForwardDist_mm );
                                g_navigationStatus = NS_SEARCH_RIGHT_WALL;
                            }

                        }
                        else
                        {
                            if(robot.bumpLeft())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                cout<<"\n\n\t ERROR: should not hit a Left Wall while in NS_PROBE_RIGHT_WALL .... \n"<<endl;

                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state
                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                g_navigationStatus = NS_SEARCHING;
                            }
                            else if(robot.bumpRight())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                                if(wallSigMgr.isNoWallSignal())
                                {
                                    cout<<"\n\t #### inside NS_PROBE_RIGHT_WALL :: NO WALL SIGNAL :: START SECOND LEVEL PROBING..."<<endl;
                                    isProbeRightWall_SecondTest = true;
                                    backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm ); // shegufta: instead of declearing a new variable for forwardTimeSlot, to keep thing simple, I have just used backupTimeSlot

                                }
                                else
                                {// if there is wall signal, do the pre-survay thing
                                    //TODO: adjust the threshold

                                    g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);
                                    //backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );  // it is not needed for NS_MOVE_AWAY_FROM_WALL
                                    rotationTimeSlot = REVERSE_ROTATION_TIME_SLOT;
                                    g_navigationStatus = NS_MOVE_AWAY_FROM_WALL;

                                }
                            }
                            else
                            {
                                if ((!wallSigMgr.isNoWallSignal())&&( ALIGNMENT_THRESHOLD < surveyManagerPtr->getSignalStrength(wallSignal) ))
                                {
                                    g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                                    rotationLimiter=0;
                                    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                    g_navigationStatus = NS_FOLLOW_WALL;
                                    consecutiveOperation = 0;
                                    backupTimeSlot = 0;
                                    alignLeft = 0;
                                    alignRight = 0;

                                }
                            }

                            robot.sendDriveCommand(SEARCHING_SPEED, SEARCH_PROBE_WALL_RADIOUS);


                        }





                        break;
                    }

                    case NS_SEARCH_FRONT_WALL:
                    {
                        current_state_slotCount++;
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

                                // TODO:: take care now

                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                                backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                g_navigationStatus = NS_SEARCHING;
                                break;
                            }
                            else if(robot.bumpRight())
                            {
                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state
                                current_state_slotCount = 0;

                                if(wallSigMgr.isNoWallSignal())
                                {
                                    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                    backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

                                    if(NULL != surveyManagerPtr)
                                    {
                                        delete surveyManagerPtr;
                                        surveyManagerPtr = NULL;
                                    }

                                    g_navigationStatus = NS_SURVEY;
                                    NS_SURVEY_ISwallAvgHighValueSeen = false;
                                    current_state_slotCount = 0;

                                }
                                else
                                {
                                    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                    backupTimeSlot = calculateTimeSlot(sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                    g_navigationStatus = NS_PRE_SURVEY;
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
                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state
                            current_state_slotCount = 0;

                            g_navigationStatus = NS_ROTATE_RIGHT_AND_SEARCH;
                            break;
                        }

                        break;
                    }

                }

            }


            //cout << "Wall signal " << wallSignal << "     sleep time "<<sleepTimeMS << "    navStatus = " << g_navigationStatus<< endl;

            this_thread::sleep_for(chrono::milliseconds(sleepTimeMS));

        }

        cout << "Play button pressed, stopping Robot" << endl;
        robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);

        g_printPositionLog();

    }
    //catch(boost::thread_interrupted&)
    catch(exception ex)
    {
    }



}

int main ()
{

    g_stateNameMap[NS_SEARCHING] = "NS_SEARCHING";
    g_stateNameMap[NS_PRE_SURVEY] = "NS_PRE_SURVEY";
    g_stateNameMap[NS_MOVE_AWAY_FROM_WALL] = "NS_MOVE_AWAY_FROM_WALL";
    g_stateNameMap[NS_SURVEY] = "NS_SURVEY";
    g_stateNameMap[NS_POST_SURVEY_ALIGN] = "NS_POST_SURVEY_ALIGN";
    g_stateNameMap[NS_FOLLOW_WALL] = "NS_FOLLOW_WALL";
    g_stateNameMap[NS_ROTATE_RIGHT_AND_SEARCH] = "NS_ROTATE_RIGHT_AND_SEARCH";
    g_stateNameMap[NS_SEARCH_FRONT_WALL] = "NS_SEARCH_FRONT_WALL";
    g_stateNameMap[NS_SEARCH_RIGHT_WALL] = "NS_SEARCH_RIGHT_WALL";
    g_stateNameMap[NS_PROBE_RIGHT_WALL] = "NS_PROBE_RIGHT_WALL";





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