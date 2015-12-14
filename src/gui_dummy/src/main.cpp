#include "ros/ros.h"
#include <ncurses.h>
#include <stdlib.h>     /* atof */
#include <sstream>
#include "gui_dummy.hpp"

//! size of terminal window
uint rows,cols;

enum{
    WHITE=1,
    GREEN,
    RED,
    BLUE
};

void printMessage(uint row, uint col, char* msg, uint color){
    mvprintw(row,col,"%s", msg);
    mvchgat(row, col, strlen(msg), A_BOLD, color, NULL);
    
}

void print(uint row, uint startcol, uint length, const char* s){
    for (uint i=startcol;i<startcol+length;i++){
        mvprintw(row,i,"%s",s);
    }
}

void clearAll(){
    for(uint i=0;i<rows;i++){
        print(i,0,cols," ");
    }
}

vector<float> parseForFloatValues(char* floatstring){
    stringstream ss;
    vector<float> setpoints;
    float sp;
    ss << floatstring;
    while(ss){
        ss >> sp;
        setpoints.push_back(sp);
    }
    return setpoints;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gui_dummy");
    
    GUI gui;
    
    ROS_INFO("gui dummy initialized");
    
    //! start ncurses mode
    initscr();
    //! Start color functionality	
    start_color();
    init_pair(WHITE, COLOR_WHITE, COLOR_BLACK);
    init_pair(GREEN, COLOR_GREEN, COLOR_BLACK);
    init_pair(RED, COLOR_RED, COLOR_BLACK);
    init_pair(BLUE, COLOR_BLUE, COLOR_BLACK);
    
    //! get the size of the terminal window
    getmaxyx(stdscr,rows,cols);    
    
    //! standard query messages
    char welcomestring[] = "commandline tool gui dummy";
    char commandstring[] = "[0]position, [1]velocity, [2]force, [3]initialize motors, [9]exit";
    char motorstring[] = "which motor [0-24]?";
    char motorsstring[] = "which motors do you want to initialize [0-24]?";
    char sampleratestring[] = "sampleRate [ms]?";
    char setpointsstring[] = "set points (rad) ?";
    char setvelsstring[] = "set velocities (rad/s) ?";
    char setforcesstring[] = "set forces (N) ?";
    char messagepublishedstring[] = "message published";
    char messagenotpublishedstring[] = "message not published";
    char inititializestring[] = "initialize motors published";
    char quitstring[] = " [hit q to quit]";
    char byebyestring[] = "BYE BYE!";
    
    print(0,0,cols,"-");
    printMessage(1,0,welcomestring,WHITE);
    print(2,0,cols,"-");
    print(6,0,cols,"-");
    printMessage(3,0,commandstring,WHITE);
    refresh();
    
    char cmd;
    float pos;
    uint motor, sampleRate;
    char floatstring[100], intstring[10];
    vector<float> floats;
    vector<unsigned char> enable;
    
    do{
        print(4,0,cols," ");
        print(5,0,cols," ");
        refresh();
        cmd = mvgetch(4,0);
        switch (cmd){
            case '0':
                printMessage(4,0,motorstring,BLUE);
                mvgetnstr(5,0,intstring,2);
                refresh();
                motor = atoi(intstring);
                print(4,0,cols," ");
                print(5,0,cols," ");
                
                printMessage(4,0,sampleratestring,BLUE);
                mvgetnstr(5,0,intstring,10);
                refresh();
                sampleRate = atoi(intstring);
                print(4,0,cols," ");
                print(5,0,cols," ");
                
                printMessage(4,0,setpointsstring,BLUE);
                refresh();
                mvgetnstr(5,0,floatstring,100);
                print(4,0,cols," ");
                print(5,0,cols," ");
                
                if(gui.sendTrajectory(motor, sampleRate, 0, parseForFloatValues(floatstring))){
                    printMessage(5,0,messagepublishedstring,GREEN);
                }else{
                    printMessage(5,0,messagenotpublishedstring,RED);
                }
                
                refresh();
                usleep(1000000);
                break;
            case '1':
                printMessage(4,0,motorstring,BLUE);
                mvgetnstr(5,0,intstring,2);
                refresh();
                motor = atoi(intstring);
                print(4,0,cols," ");
                print(5,0,cols," ");
                
                printMessage(4,0,sampleratestring,BLUE);
                mvgetnstr(5,0,intstring,10);
                refresh();
                sampleRate = atoi(intstring);
                print(4,0,cols," ");
                print(5,0,cols," ");
                
                printMessage(4,0,setvelsstring,BLUE);
                refresh();
                mvgetnstr(5,0,floatstring,100);
                print(4,0,cols," ");
                print(5,0,cols," ");
                
                if(gui.sendTrajectory(motor, sampleRate, 0, parseForFloatValues(floatstring))){
                    printMessage(5,0,messagepublishedstring,GREEN);
                }else{
                    printMessage(5,0,messagenotpublishedstring,RED);
                } 
                
                refresh();
                usleep(1000000);
                break;
            case '2':
                printMessage(4,0,motorstring,BLUE);
                mvgetnstr(5,0,intstring,2);
                refresh();
                motor = atoi(intstring);
                print(4,0,cols," ");
                print(5,0,cols," ");
                
                printMessage(4,0,sampleratestring,BLUE);
                mvgetnstr(5,0,intstring,10);
                refresh();
                sampleRate = atoi(intstring);
                print(4,0,cols," ");
                print(5,0,cols," ");
                
                printMessage(4,0,setforcesstring,BLUE);
                refresh();
                mvgetnstr(5,0,floatstring,100);
                print(4,0,cols," ");
                print(5,0,cols," ");
                
                if(gui.sendTrajectory(motor, sampleRate, 0, parseForFloatValues(floatstring))){
                    printMessage(5,0,messagepublishedstring,GREEN);
                }else{
                    printMessage(5,0,messagenotpublishedstring,RED);
                } 
                
                refresh();
                usleep(1000000);
                break;
            case '3':
                printMessage(4,0,motorsstring,BLUE);
                refresh();
                mvgetnstr(5,0,floatstring,100);
                floats = parseForFloatValues(floatstring);
                enable.clear();
                for(uint i=0;i<floats.size();i++){
                    enable.push_back(1);
                }
                gui.initRequest(enable);
                print(4,0,cols," ");
                print(5,0,cols," ");
                printMessage(5,0,inititializestring,GREEN);
                refresh();
                usleep(1000000);
                break;
            case '9':
                clearAll();
                printMessage(rows/2,cols/2-strlen(byebyestring)/2,byebyestring,WHITE);
                refresh();
                usleep(500000);
        }
        
    }while( cmd != '9');
    endwin();
    
    return 0;
}
