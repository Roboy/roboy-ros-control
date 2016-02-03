#pragma once

//#include "plot.hpp"
#include <ncurses.h>
#include <vector>
#include <string>

using namespace std;

enum COLORS{
	CYAN=1,
	RED,
	GREEN,
	YELLOW
};

#define ESCAPE_KEY 27
#define ENTER_KEY 10

class NCursesInterface{
public:
	NCursesInterface(){
		//! start ncurses mode
		initscr();
		//! Start color functionality
		start_color();
		init_pair(CYAN, COLOR_CYAN, COLOR_BLACK);
		init_pair(RED, COLOR_RED, COLOR_BLACK);
		init_pair(GREEN, COLOR_GREEN, COLOR_BLACK);
		init_pair(YELLOW, COLOR_YELLOW, COLOR_BLACK);
		//! get the size of the terminal window
		getmaxyx(stdscr,rows,cols);
		//! enable special keys
		keypad(stdscr, TRUE);

		print(0,0,cols,"-");
		printMessage(1,0,"ROBOY hardware interface");
		print(2,0,cols,"-");
		printMessage(3,0," ");
		noecho();

//		m_plot = new Plot(10);
	}
	~NCursesInterface(){
		clearAll();
		printMessage(rows/2,cols/2-strlen("BYE BYE")/2,"BYE BYE");
		refresh();
		usleep(1000000);
		endwin();
	}
	void MotorsInitialized(){
		clearAll(4);
		printMessage(4,0,"motors initialized", YELLOW );
	}
	void statusMotor(uint ganglion, uint motor, uint status){
		uint row = ganglion*4;
		uint col = motor*10;
		printMessage(row+3,col,"----------");
		printMessage(row+6,col,"----------");
		char name[10];
		sprintf(name,"motor%d", ganglion*4+motor);
		printMessage(row+4,col,name);
		switch(status){
			case 0:
				printMessage(row+5,col,"not ready", RED);
				break;
			case 1:
				printMessage(row+5,col,"ready", GREEN);
				break;
//			case 2:
//				printMessage(row*4,col*10,"processing", YELLOW);
//				break;
		}
		refresh();
	}
	void printMessage(uint row, uint col, const char* msg){
		mvprintw(row,col,"%s", msg);
		refresh();
	}
	void printMessage(uint row, uint col, const char* msg, uint color){
		mvprintw(row,col,"%s", msg);
		mvchgat(row, col, strlen(msg), A_BOLD, color, NULL);
		refresh();
	}
	void print(uint row, uint col, uint nr){
		mvprintw(row,col,"%d", nr);
		refresh();
	}
	void print(uint row, uint startcol, uint length, const char* s){
		for (uint i=startcol;i<startcol+length;i++){
			mvprintw(row,i,"%s",s);
		}
		refresh();
	}
	void clearAll(){
		for(uint i=0;i<rows;i++){
			print(i,0,cols," ");
		}
		refresh();
	}
	void clearAll(uint row){
		for(uint i=row;i<rows;i++){
			print(i,0,cols," ");
		}
		refresh();
	}
//	Plot *m_plot;
private:
	uint rows, cols;
	char inputstring[200];
	char m_zeros[200]={'\0'};
};
