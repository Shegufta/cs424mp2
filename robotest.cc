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

enum NS_SEARCHING_SUBSTATE
{
    NS_SEARCH_FOR_HIGH_GROUND,
    NS_SEARCH_MOVE_HIGH_GROUND,
    NS_SEARCH_REPOSITION
};

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

    int getCurrentSignalStatus()
    {// 1 means larger, 0 means equal, -1 means smaller
        int del = wallSignalHistoryArray[WALL_SIGNAL_HISTORY_SIZE-1] - wallSignalHistoryArray[WALL_SIGNAL_HISTORY_SIZE-2];

        if(del == 0)
            return 0;
        else if(0 < del)
            return 1;
        else if(del < 0)
            return -1;
    }

    bool isWallSignalStrongEnough()
    {
        return ( WALL_SENSOR_MIN < wallSignalHistoryArray[WALL_SIGNAL_HISTORY_SIZE-1] );
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



int g_sleepTimeMS = 15;
int g_stopForPhotoTimeMS = g_sleepTimeMS * 5;
bool g_Is_Taking_Picture = false;
NAVIGATION_STATUS g_navigationStatus;


void navigate(void* _robot)
{
    Create robot = *((Create*) _robot);




#if false

    const int RotationSpeed = 300; /// @ 300mmps, and sleep interval 15ms, it takes approx 184 slot for a 360 degree movement
    int temp_sleepTimeMS = 15;
    short wallSignal_temp ;

    //int sleepMS = 10000;
    //robot.sendDriveCommand (200, Create::DRIVE_STRAIGHT);
    //robot.sendDriveCommand (200, Create::DRIVE_INPLACE_CLOCKWISE);
    //robot.sendDriveCommand (100, -200);
    //cout<<"start sleep"<<endl;
    //this_thread::sleep_for(chrono::milliseconds(sleepMS));
    //cout<<"end sleep"<<endl;
    //robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);

    const int _FOLLOW_WALL_SPEED = 200;
    const int _ANTICLOCK_WISE_RADIOUS = 400;
    int counter = 0;
    int findMax = -1;
    bool readWallOnly = false;
    while (!robot.playButton ())
    {
        wallSignal_temp = robot.wallSignal();

        //robot.sendDriveCommand(_FOLLOW_WALL_SPEED, _ANTICLOCK_WISE_RADIOUS);



        /*
        if(findMax == wallSignal_temp &&  (100 < findMax) )
        {

            robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
            cout <<"#### max wall signal "<<wallSignal_temp<<endl;
            readWallOnly = true;
            return;
        }

        if(!readWallOnly)
        {
            if(findMax < wallSignal_temp)
                findMax = wallSignal_temp;
            robot.sendDriveCommand(RotationSpeed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
        }

        robot.sendDriveCommand(RotationSpeed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
        */

        counter++;
        cout <<"counter = "<<counter << "  |  Wall signal " << wallSignal_temp << endl;
        this_thread::sleep_for(chrono::milliseconds(temp_sleepTimeMS));




    }

    cout << "Play button pressed, stopping Robot" << endl;
    robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);



    return;
#endif

    const double ALIGNMENT_THRESHOLD = 0.9;
    const double LOWER_BOUND_OF_VALID_THRESHOLD = 0.5;

    const double INIT_OUTOF_CONTROL_THRESHOLD = 0.4;
    double OUTOF_CONTROL_THRESHOLD = INIT_OUTOF_CONTROL_THRESHOLD;


    const int RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT = 20;

    const int n_SEARCHING_ROTATION_SPEED = 300; // NOTE: at 300mmps and 15ms sleep interval, it takes approx 184 slot for a 360 degree rotation
    const int n_SEARCHING_STRAIGHT_SPEED = 200;
    const int n_SEARCH_FRONT_WALL_SLOW_BACKUP = 100; // if found front wall, then backup slowly

    const int SEARCHING_SPEED = 100;
    const int MID_BACKUP_DIST_mm = 10;
    const int SEARCH_R_WALL_ForwardDist_mm = 120;
    const int NS_SURVEY_SLOT_MAX = 70; // NOTE: at n_SEARCHING_ROTATION_SPEED = 300mmps and 15ms sleep interval, it takes approx 184 slot for a 360 degree rotation/// We will scan max 135 degree, henc our max slot will be approx 70....   Note, ((360/8)*3) = 135
    const int n_FOLLOW_WALL_CLOCK_WISE_RADIOUS = -50;
    const int n_FOLLOW_WALL_ANTICLOCK_WISE_RADIOUS = -1 * n_FOLLOW_WALL_CLOCK_WISE_RADIOUS;
    const int SEARCH_RIGHT_WALL_RADIOUS = -185;  // radious 185 is ok when the searching speed is 100mmps



    const int SEARCH_PROBE_WALL_RADIOUS = -280;

    const int REVERSE_ROTATION_RADIOUS = 100;
    const int REVERSE_ROTATION_TIME_SLOT = 75;

    const int SEARCH_F_WALL_BACKUP_DIST_mm = 80;
    const int SEARCH_F_WALL_RADIOUS = 50;
    const int FRONT_WALL_SEARCH_ROTATION_TIME_SLOT = 80;

    const int FOLLOW_WALL_SPEED = 200;

    const int CLIFF_SENSOR_THRESHOLD = 10;
    const int WALL_SENSOR_MIN = 4;


    const int FOLLOW_WALL_CHECK_SIGNAL_INTERVAL = 4;

    //////////////////////////
    const int SKIP_DUE_TO_OVERCURRENT_SLOT = 6;
    const int MAX_OVERCURRENT_SAFE_SLOT = 3;
    int overcurrentSlotCounter = 0;

    const int PROBE_RIGHTWALL_ANTCLOCK_SMALL_ROTATION = 25;
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
    int n_consecutiveDecreaseInPostSurveyAlign = 0;
    double n_prevSignalStrengthInPostSurveyAlign = 0;
    bool n_IsfinalAdjustRotationInPostSurveyAlign = 0;
    const int POST_SURVEY_DECREMENT_OR_EQUAL_THRESHOLD = 2;

    int probRightWall_antiClockWise_smallRotationSlot;

    bool isProbeRightWall_SecondTest = false;


    SurveyManager *surveyManagerPtr = NULL;
    WallSignalManager wallSigMgr(FOLLOW_WALL_CHECK_SIGNAL_INTERVAL, WALL_SENSOR_MIN);

    g_navigationStatus = NS_SEARCHING;
    current_state_slotCount = 0;
    surveyManagerPtr = NULL;

    //////////////////////__new_strategy_13Nov2016////////////////////////
    //short prevWallSignal = 0;
    bool n_isSurveyDirClockWise = false;
    NS_SEARCHING_SUBSTATE n_SEARCHING_SUBSTATE = NS_SEARCH_FOR_HIGH_GROUND;
    int n_consecutiveEventCounter = 0;
    int n_consecutiveClimbUpCounter = 0;
    int n_consecutiveClimbDownCounter = 0;
    const int n_CONSECUTIVE_UP_COUNT = 3;
    const int n_CONSECUTIVE_DOWN_COUNT = 3;

    //////////////////////////////////////////////////////////////////////


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
                    //cout <<"IsCliffSensorsOK = "<<IsCliffSensorsOK <<" | IsWheelDropOk = "<<endl;
                    robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                    // TODO: turn on alarm
                }


                if(!IsOvercurrentOK)
                {
                    ++overcurrentSlotCounter;

                    if(MAX_OVERCURRENT_SAFE_SLOT < overcurrentSlotCounter)
                    {
                        //cout <<" | IsOvercurrentOK = "<<IsOvercurrentOK<< " | overcurrentSlotCounter= "<<overcurrentSlotCounter<<endl;
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
                        current_state_slotCount++; // TODO: initiate it to zero

                        if(0 < backupTimeSlot)
                        {
                            backupTimeSlot--;
                            robot.sendDriveCommand (-n_SEARCHING_STRAIGHT_SPEED, Create::DRIVE_STRAIGHT);

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

                            backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, n_SEARCH_FRONT_WALL_SLOW_BACKUP, MID_BACKUP_DIST_mm );

                            cout<<"NS_SEARCHING -> BUMP LEFT : next State ->  NS_SEARCH_FRONT_WALL";

                            g_navigationStatus = NS_SEARCH_FRONT_WALL;
                        }
                        else if(robot.bumpRight())
                        {
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, n_SEARCHING_STRAIGHT_SPEED, MID_BACKUP_DIST_mm );
                            cout<<"NS_SEARCHING -> BUMP RIGHT : next State ->";
                            if(wallSigMgr.isNoWallSignal())
                            {
                                cout << "\tGO TO -> NS_SURVEY"<<endl;

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
                                g_navigationStatus = NS_PRE_SURVEY;
                            }



                        }
                        else
                        {
                            robot.sendDriveCommand (n_SEARCHING_STRAIGHT_SPEED, Create::DRIVE_STRAIGHT);
                        }


                        break;
                    }

                    case NS_PRE_SURVEY:
                    {
                        current_state_slotCount++;

                        if(0 < backupTimeSlot)
                        {
                            robot.sendDriveCommand (-n_SEARCHING_STRAIGHT_SPEED, Create::DRIVE_STRAIGHT);
                            backupTimeSlot--;

                            if(0 == backupTimeSlot)
                                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                        }
                        else
                        {
                            robot.sendDriveCommand(n_SEARCHING_ROTATION_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);

                            if(wallSigMgr.isNoWallSignal())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                                if(NULL != surveyManagerPtr)
                                {
                                    delete surveyManagerPtr;
                                    surveyManagerPtr = NULL;
                                }

                                g_navigationStatus = NS_SURVEY;
                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                rotationLimiter = 0;
                                cout <<"\t\t\t moving to NS_SURVEY"<<endl;
                            }
                        }

                        break;
                    }

                    case NS_SURVEY: {// Condition: when it will enter the NS_SURVEY mode FOR THE FIRST TIME, the condition  [ws_getAverage(wallSignal) < WALL_SENSOR_MIN)] will be true
                        //CONDITION FOR THE FIRST TIME : NS_SURVEY_ISwallAvgHighValueSeen = false;
                        //rotationLimiter = 0;
                        // Make sure, surveyManagerPtr is a new instance, before moving to this state, free it and set it to null

                        // SET n_isSurveyDirClockWise
                        // n_SEARCHING_SUBSTATE = NS_SEARCH_FOR_HIGH_GROUND;
                        // n_consecutiveEventCounter = 0

                        //n_consecutiveClimbUpCounter = 0
                        //n_consecutiveClimbDownCounter = 0

                        ++current_state_slotCount;

                        if (0 < backupTimeSlot)
                        {
                            robot.sendDriveCommand(-n_SEARCHING_STRAIGHT_SPEED, Create::DRIVE_STRAIGHT);
                            backupTimeSlot--;

                            if (0 == backupTimeSlot)
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                        }
                        else if (rotationLimiter < NS_SURVEY_SLOT_MAX)
                        {
                            ++rotationLimiter;


                            int currentSignalStatus = wallSigMgr.getCurrentSignalStatus();

                            switch(n_SEARCHING_SUBSTATE)
                            {
                                case NS_SEARCH_FOR_HIGH_GROUND:
                                {

                                    if(wallSigMgr.isWallSignalStrongEnough())
                                    {

                                        if(1 == currentSignalStatus)
                                        {
                                            ++n_consecutiveClimbUpCounter;
                                            n_consecutiveClimbDownCounter = 0;

                                            cout<<" NS_SEARCH_FOR_HIGH_GROUND wallSig = "<<wallSignal <<" | currentSignalStatus = "<<currentSignalStatus <<" | n_consecutiveClimbUPCounter = "<<n_consecutiveClimbUpCounter<<endl;

                                            if(n_CONSECUTIVE_UP_COUNT == n_consecutiveClimbUpCounter)
                                            {
                                                cout<<"\t Move To NS_SEARCH_MOVE_HIGH_GROUND"<<endl;
                                                n_consecutiveClimbUpCounter = 0;
                                                n_SEARCHING_SUBSTATE = NS_SEARCH_MOVE_HIGH_GROUND;
                                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                                break;
                                            }



                                        }
                                        else if(0 == currentSignalStatus)
                                        {

                                        }
                                        else if(-1 == currentSignalStatus)
                                        {
                                            ++n_consecutiveClimbDownCounter;
                                            n_consecutiveClimbUpCounter = 0;

                                            cout<<" NS_SEARCH_FOR_HIGH_GROUND wallSig = "<<wallSignal <<" | currentSignalStatus = "<<currentSignalStatus <<" | n_consecutiveClimbDOWNCounter = "<<n_consecutiveClimbDownCounter<<endl;

                                            if(n_CONSECUTIVE_DOWN_COUNT == n_consecutiveClimbDownCounter)
                                            {// it is climbing DOWN.... so change the direction and change state
                                                if(n_isSurveyDirClockWise)
                                                    n_isSurveyDirClockWise = false;
                                                else
                                                    n_isSurveyDirClockWise = true;

                                                n_consecutiveClimbDownCounter = 0;
                                                n_SEARCHING_SUBSTATE = NS_SEARCH_MOVE_HIGH_GROUND;
                                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                                break;
                                            }
                                        }


                                    }
                                    else
                                    {
                                        n_consecutiveClimbDownCounter = 0;
                                        n_consecutiveClimbUpCounter = 0;

                                    }

                                    if(n_isSurveyDirClockWise)
                                        robot.sendDriveCommand(n_SEARCHING_ROTATION_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
                                    else
                                        robot.sendDriveCommand(n_SEARCHING_ROTATION_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);


                                    break;
                                }

                                case NS_SEARCH_MOVE_HIGH_GROUND:
                                {
                                    if(-1==currentSignalStatus)
                                    {
                                        ++n_consecutiveClimbDownCounter;
                                        cout<<" NS_SEARCH_FOR_HIGH_GROUND wallSig = "<<wallSignal <<" | currentSignalStatus = "<<currentSignalStatus <<" | n_consecutiveClimbDOWNCounter = "<<n_consecutiveClimbDownCounter<<endl;
                                        if(n_CONSECUTIVE_DOWN_COUNT == n_consecutiveClimbDownCounter)
                                        {
                                            cout<<"\t Move To NS_SEARCH_REPOSITION"<<endl;
                                            if(n_isSurveyDirClockWise)
                                                n_isSurveyDirClockWise = false;
                                            else
                                                n_isSurveyDirClockWise = true;

                                            //n_consecutiveClimbDownCounter = 0;  // NOTE  : DONOT make it zero... rather use it as a counter for the next state
                                            n_SEARCHING_SUBSTATE = NS_SEARCH_REPOSITION;
                                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                            break;
                                        }
                                    }
                                    else
                                    {//large or equal
                                        cout<<" NS_SEARCH_FOR_HIGH_GROUND wallSig = "<<wallSignal<<endl;
                                        n_consecutiveClimbDownCounter =  0;
                                    }

                                    if(n_isSurveyDirClockWise)
                                        robot.sendDriveCommand(n_SEARCHING_ROTATION_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
                                    else
                                        robot.sendDriveCommand(n_SEARCHING_ROTATION_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

                                    break;
                                }
                                case NS_SEARCH_REPOSITION:
                                {
                                    cout<<" NS_SEARCH_REPOSITION wallSig = "<<wallSignal <<endl;

                                    if(0 < n_consecutiveClimbDownCounter)
                                    {
                                        --n_consecutiveClimbDownCounter;

                                        if(n_isSurveyDirClockWise)
                                            robot.sendDriveCommand(n_SEARCHING_ROTATION_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
                                        else
                                            robot.sendDriveCommand(n_SEARCHING_ROTATION_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);


                                    }
                                    else if (0 == n_consecutiveClimbDownCounter)
                                    {
                                        cout <<"\t\t\t\t  NS_SEARCH_REPOSITION WALL SIGNAL MAX = "<<wallSignal<<endl;



                                        cout<<"bye"<<endl;//TODO: remove
                                        robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);//TODO: remove
                                        return;//TODO: remove


                                        g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                                        rotationLimiter=0;
                                        robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                        g_navigationStatus = NS_FOLLOW_WALL;
                                        consecutiveOperation = 0;
                                        backupTimeSlot = 0;
                                        alignLeft = 0;
                                        alignRight = 0;
                                        cout <<"\t\t\t moving to NS_FOLLOW_WALL"<<endl;

                                        robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                        break;
                                    }

                                    break;
                                }
                            }




                            /*

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
                                cout <<"\t\t\t moving to NS_POST_SURVEY_ALIGN"<<endl;


                            }else {
                                robot.sendDriveCommand(n_SEARCHING_ROTATION_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                            }
                             */
                        }
                        else
                        {

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            g_navigationStatus = NS_ROTATE_RIGHT_AND_SEARCH;
                            rotationLimiter = 0;
                            n_consecutiveDecreaseInPostSurveyAlign = 0;
                            n_prevSignalStrengthInPostSurveyAlign = -1.0;
                            n_IsfinalAdjustRotationInPostSurveyAlign = false;
                        }

                        break;
                    }
                    case NS_POST_SURVEY_ALIGN:
                    {//condition rotationLimiter = 0;

                        current_state_slotCount++;



                        double signalStrength = surveyManagerPtr->getSignalStrength(wallSignal);




                        if (rotationLimiter < NS_SURVEY_SLOT_MAX)
                        {
                            rotationLimiter++;

                            if((0.5 < signalStrength)&&(signalStrength <= n_prevSignalStrengthInPostSurveyAlign) )// NOTE:: here i am counting the equal signal strength also...
                                ++n_consecutiveDecreaseInPostSurveyAlign;
                            else
                                n_consecutiveDecreaseInPostSurveyAlign = 0;

                            n_prevSignalStrengthInPostSurveyAlign = signalStrength;

                            cout <<"\tsurveyManagerPtr->getSignalStrength(wallSignal) = "<<signalStrength<< "   | wallSignal = "<<wallSignal<<"  | ALIGNMENT_THRESHOLD = "<<ALIGNMENT_THRESHOLD <<"  | n_consecutiveDecreaseInPostSurveyAlign"<<n_consecutiveDecreaseInPostSurveyAlign<<endl;



                            if ( (signalStrength < ALIGNMENT_THRESHOLD) && ( POST_SURVEY_DECREMENT_OR_EQUAL_THRESHOLD != n_consecutiveDecreaseInPostSurveyAlign) )
                                robot.sendDriveCommand(n_SEARCHING_ROTATION_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
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
                                cout <<"\t\t\t moving to NS_FOLLOW_WALL"<<endl;
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
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            cout << "inside NS_ROTATE_RIGHT_AND_SEARCH : next state NS_SEARCH_FRONT_WALL"<<endl;

                            backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, SEARCH_F_WALL_BACKUP_DIST_mm );
                            g_navigationStatus = NS_SEARCH_FRONT_WALL;
                            // TODO: handle now ... start searching for front wall
                            cout <<"\t\t\t TODO: handle corner case... it should not be a problem for mp2... inside NS_ROTATE_RIGHT_AND_SEARCH"<<endl;


                        }
                        else if(robot.bumpRight())
                        {
                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            if(wallSigMgr.isNoWallSignal())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                cout << "\tinside NS_ROTATE_RIGHT_AND_SEARCH, GO TO -> NS_SURVEY"<<endl;
                                backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

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
                                backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
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
                                    backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );

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
                                    backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
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

                            robot.sendDriveCommand(-n_SEARCHING_STRAIGHT_SPEED, REVERSE_ROTATION_RADIOUS);

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

                                backupTimeSlot = 0;//calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm ); // we have already backed up
                                NS_SURVEY_ISwallAvgHighValueSeen = false;
                                rotationLimiter = 0;
                                g_navigationStatus = NS_SURVEY;

                            }
                            else
                            {
                                cout << "\tGO TO -> NS_PRE_SURVEY"<<endl;
                                backupTimeSlot = 0;//calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );// we have already backed up
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




                            cout<<"good bye 1"<<endl;

                            return; // TODO: test purpose... remove




                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            cout << "inside NS_FOLLOW_WALL : next state NS_SEARCH_FRONT_WALL"<<endl;

                            backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, n_SEARCH_FRONT_WALL_SLOW_BACKUP, SEARCH_F_WALL_BACKUP_DIST_mm );
                            g_navigationStatus = NS_SEARCH_FRONT_WALL;
                        }
                        else if(robot.bumpRight())
                        {
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);


                            ///cout<<"good bye 2"<<endl;
                            //return;  // TODO: test purpose... remove




                            cout<<"BUMP RIGHT inside NS_FOLLOW_WALL"<<endl;



                            g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                            if(wallSigMgr.isNoWallSignal())
                            {

                                rotationTimeSlot = RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT;
                                backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm ); // shegufta: instead of declearing a new variable for forwardTimeSlot, to keep thing simple, I have just used backupTimeSlot
                                forwardTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, SEARCH_R_WALL_ForwardDist_mm );
                                g_navigationStatus = NS_SEARCH_RIGHT_WALL;
                                cout << "inside NS_FOLLOW_WALL : next state NS_SEARCH_RIGHT_WALL"<<endl;
                            }
                            else
                            {// if there is wall signal, do the pre-survay thing
                                //TODO: adjust the threshold



                                cout << "BYE 2222"<<endl;
                                return;



                                backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, n_SEARCHING_STRAIGHT_SPEED, MID_BACKUP_DIST_mm );
                                cout << "\tGO TO -> NS_PRE_SURVEY"<<endl;
                                g_navigationStatus = NS_PRE_SURVEY;






                                /*
                                rotationTimeSlot = REVERSE_ROTATION_TIME_SLOT;
                                g_navigationStatus = NS_MOVE_AWAY_FROM_WALL;
                                cout << "inside NS_FOLLOW_WALL : next state NS_MOVE_AWAY_FROM_WALL"<<endl;
                                */

                            }

                        }
                        else

                        {
                            double signalStrength = -1234;//surveyManagerPtr->getSignalStrength(wallSignal);

                            cout <<"wallSignal = " << wallSignal <<"     |   surveyManagerPtr->getSignalStrength(wallSignal)  = " << signalStrength <<endl;

                            if(wallSigMgr.isNoWallSignal())
                            {
                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                                isProbeRightWall_SecondTest = false;
                                goBackPreviousPosition = 0;
                                g_navigationStatus = NS_PROBE_RIGHT_WALL;
                                cout << "\t\t\t\tinside NS_FOLLOW_WALL : next state NS_PROBE_RIGHT_WALL"<<endl;
                                cout <<"\t\t\t KILL before Probe Right Wall"<<endl;



                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT); // TODO: REMOVE
                                return; // TODO: removve

                            }/*
                            else if(signalStrength < LOWER_BOUND_OF_VALID_THRESHOLD)
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state
                                backupTimeSlot = 0;//calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );// we have already backed up
                                g_navigationStatus = NS_PRE_SURVEY;
                                cout <<"\t\t\t moving to NS_PRE_SURVEY"<<endl;
                            }*/
                            else
                            {
                                if( 0 != alignLeft )
                                {
                                    robot.sendDriveCommand(FOLLOW_WALL_SPEED, n_FOLLOW_WALL_ANTICLOCK_WISE_RADIOUS);
                                    cout<<"\t\tLEFT"<<endl;
                                    if(!wallSigMgr.isIncreasing())
                                        alignLeft = 0;

                                }
                                else if((0 != alignRight))
                                {
                                    robot.sendDriveCommand(FOLLOW_WALL_SPEED, n_FOLLOW_WALL_CLOCK_WISE_RADIOUS);
                                    cout<<"\t\tRIGHT"<<endl;
                                    if(wallSigMgr.isIncreasing())
                                        alignRight = 0;
                                }
                                else
                                {
                                    robot.sendDriveCommand(FOLLOW_WALL_SPEED, Create::DRIVE_STRAIGHT);
                                    /*
                                    if(consecutiveOperation < FOLLOW_WALL_CHECK_SIGNAL_INTERVAL)
                                    {
                                        ++consecutiveOperation;

                                        robot.sendDriveCommand(FOLLOW_WALL_SPEED, Create::DRIVE_STRAIGHT);
                                        cout<<"\t\tSTRAIGHT"<<endl;

                                    }
                                    else
                                    {
                                        consecutiveOperation=0;

                                        robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                        cout<<"\t\tSTOP"<<endl;

                                        if( (LOWER_BOUND_OF_VALID_THRESHOLD <=signalStrength ) && (signalStrength < OUTOF_CONTROL_THRESHOLD))
                                        {
                                            cout <<"\t alighRight 4"<<endl;
                                            alignRight = 4;
                                        }
                                        else
                                        {
                                            if (wallSigMgr.isIncreasing()) {
                                                alignLeft = 4;
                                                cout <<"\t alighLeft 1"<<endl;
                                            }
                                            else {
                                                alignRight = 4;
                                                cout <<"\t alighRight 1"<<endl;
                                            }
                                        }


                                    }
                                    */

                                }






                                //cout << "\t\tSTRAIGHT"<<endl;
                                //robot.sendDriveCommand(FOLLOW_WALL_SPEED, Create::DRIVE_STRAIGHT);
                            }

                            /*

                            cout << "current_state_slotCount = "<<current_state_slotCount<<"  wallsig = "<<wallSignal << "  sigstrength = "<<surveyManagerPtr->getSignalStrength(wallSignal)<<endl;
                            if(100 == current_state_slotCount) { // TODO: test code... remove
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                return;
                            }

                            if(0 < alignLeft)
                            {
                                robot.sendDriveCommand(FOLLOW_WALL_SPEED, n_FOLLOW_WALL_ANTICLOCK_WISE_RADIOUS);
                                cout<<"\t\tLEFT"<<endl;
                                alignLeft--;
                            }
                            else if(0 < alignRight)
                            {
                                robot.sendDriveCommand(FOLLOW_WALL_SPEED, n_FOLLOW_WALL_CLOCK_WISE_RADIOUS);
                                cout<<"\t\tRIGHT"<<endl;
                                alignRight--;
                            }
                            else
                            {
                                if(consecutiveOperation < FOLLOW_WALL_CHECK_SIGNAL_INTERVAL)
                                {
                                    robot.sendDriveCommand(FOLLOW_WALL_SPEED, Create::DRIVE_STRAIGHT);
                                    cout<<"\t\tSTRAIGHT"<<endl;
                                    consecutiveOperation++;
                                }
                                else
                                {
                                    consecutiveOperation=0;

                                    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                    cout<<"\t\tSTOP"<<endl;

                                    if(wallSigMgr.isNoWallSignal())
                                    {
                                        g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                                        isProbeRightWall_SecondTest = false;
                                        goBackPreviousPosition = 0;
                                        g_navigationStatus = NS_PROBE_RIGHT_WALL;
                                        cout << "inside NS_FOLLOW_WALL : next state NS_PROBE_RIGHT_WALL"<<endl;
                                    }
                                    else
                                    {
                                        if( (LOWER_BOUND_OF_VALID_THRESHOLD <=surveyManagerPtr->getSignalStrength(wallSignal) ) && (surveyManagerPtr->getSignalStrength(wallSignal) < OUTOF_CONTROL_THRESHOLD))
                                        //if( (surveyManagerPtr->getSignalStrength(wallSignal) < OUTOF_CONTROL_THRESHOLD))
                                        {
                                            cout <<"\t alighRight 4"<<endl;
                                            alignRight = 4;
                                        }
                                        else
                                        {
                                            if (wallSigMgr.isIncreasing()) {
                                                alignLeft = 4;
                                                cout <<"\t alighLeft 1"<<endl;
                                            }
                                            else {
                                                alignRight = 4;
                                                cout <<"\t alighRight 1"<<endl;
                                            }
                                        }

                                    }


                                }

                            }
                             */

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
                            else if(0 < probRightWall_antiClockWise_smallRotationSlot)
                            {
                                --probRightWall_antiClockWise_smallRotationSlot;
                                robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                                if(probRightWall_antiClockWise_smallRotationSlot == 0)
                                    robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                            }
                            else if(0 < forwardTimeSlot)
                            {
                                forwardTimeSlot--;
                                robot.sendDriveCommand (SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            }
                            else if(rotationLimiter < NS_SURVEY_SLOT_MAX/5)
                            {
                                rotationLimiter++;

                                if(!wallSigMgr.isNoWallSignal())
                                {
                                    cout << "inside NS_PROBE_RIGHT_WALL : next state NS_PRE_SURVEY"<<endl;
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

                                cout << "inside NS_PROBE_RIGHT_WALL : next state NS_SEARCH_RIGHT_WALL"<<endl;
                                isProbeRightWall_SecondTest = false;

                                rotationTimeSlot = RIGHT_WALL_SEARCH_NEGATIVE_ROTATION_TIME_SLOT;
                                backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm ); // shegufta: instead of declearing a new variable for forwardTimeSlot, to keep thing simple, I have just used backupTimeSlot
                                //backupTimeSlot = 0; // we have already backed up
                                forwardTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, SEARCH_R_WALL_ForwardDist_mm );
                                g_navigationStatus = NS_SEARCH_RIGHT_WALL;
                            }

                        }
                        else
                        {
                            if(robot.bumpLeft())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                cout<<"\n\n\t ERROR: should not hit a Left Wall while in NS_PROBE_RIGHT_WALL .... \n"<<endl;

                                cout << "inside NS_PROBE_RIGHT_WALL : next state NS_SEARCHING"<<endl;
                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state
                                backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                g_navigationStatus = NS_SEARCHING;
                            }
                            else if(robot.bumpRight())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                                if(wallSigMgr.isNoWallSignal())
                                {
                                    cout<<"\n\t #### inside NS_PROBE_RIGHT_WALL :: NO WALL SIGNAL :: START SECOND LEVEL PROBING..."<<endl;
                                    isProbeRightWall_SecondTest = true;
                                    backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm ); // shegufta: instead of declearing a new variable for forwardTimeSlot, to keep thing simple, I have just used backupTimeSlot

                                    forwardTimeSlot = backupTimeSlot;
                                    probRightWall_antiClockWise_smallRotationSlot = PROBE_RIGHTWALL_ANTCLOCK_SMALL_ROTATION;
                                }
                                else
                                {// if there is wall signal, do the pre-survay thing
                                    //TODO: adjust the threshold

                                    cout << "inside NS_PROBE_RIGHT_WALL : next state NS_MOVE_AWAY_FROM_WALL"<<endl;
                                    g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);
                                    //backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );  // it is not needed for NS_MOVE_AWAY_FROM_WALL
                                    rotationTimeSlot = REVERSE_ROTATION_TIME_SLOT;
                                    g_navigationStatus = NS_MOVE_AWAY_FROM_WALL;

                                }
                            }
                            else
                            {
                                if ((!wallSigMgr.isNoWallSignal())&&( ALIGNMENT_THRESHOLD < surveyManagerPtr->getSignalStrength(wallSignal) ))
                                {
                                    cout << "inside NS_PROBE_RIGHT_WALL : next state NS_FOLLOW_WALL"<<endl;
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

                            robot.sendDriveCommand (-n_SEARCH_FRONT_WALL_SLOW_BACKUP, Create::DRIVE_STRAIGHT);


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


                            //////////////////////////////////////////////////////////

                            robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

                            if(0 == rotationTimeSlot)
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                            break;
                        }
                        else
                        {
                            if(robot.bumpLeft() || robot.bumpRight())
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                                g_AddPosition_RESET_current_state_slotCount(g_navigationStatus, current_state_slotCount);// add how many slot it has been spent in this particular state

                                cout<<"NS_SEARCHING -> BUMP RIGHT : next State ->";
                                if(wallSigMgr.isNoWallSignal())
                                {
                                    cout << "\tGO TO -> NS_SURVEY"<<endl;
                                    backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );


                                    if(NULL != surveyManagerPtr)
                                    {
                                        delete surveyManagerPtr;
                                        surveyManagerPtr = NULL;
                                    }

                                    NS_SURVEY_ISwallAvgHighValueSeen = false;
                                    rotationLimiter = 0;
                                    g_navigationStatus = NS_SURVEY;
                                    cout << "inside NS_SEARCH_FRONT_WALL : next state NS_SURVEY"<<endl;

                                }
                                else
                                {
                                    cout << "\tGO TO -> NS_PRE_SURVEY"<<endl;
                                    backupTimeSlot = calculateTimeSlot(g_sleepTimeMS, SEARCHING_SPEED, MID_BACKUP_DIST_mm );
                                    g_navigationStatus = NS_PRE_SURVEY;
                                    cout << "inside NS_SEARCH_FRONT_WALL : next state NS_PRE_SURVEY"<<endl;
                                }



                            }
                            else
                            {
                                robot.sendDriveCommand (SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            }

                        }

                        break;
                    }

                }

            }


            //cout << "Wall signal " << wallSignal << "     sleep time "<<g_sleepTimeMS << "    navStatus = " << g_navigationStatus<< endl;


            this_thread::sleep_for(chrono::milliseconds(g_sleepTimeMS));

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