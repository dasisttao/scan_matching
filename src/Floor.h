#ifndef FLOOR_H_
#define FLOOR_H_
#include "myInit.h"

class FLOOR{
 public:
  enum CarOnFloor
  {
    E0,
    E1,
    E2,
    R12,
    R10,
    R21
  };
  CarOnFloor caronfloor;

  // Default: car is on the floor 0
  FLOOR(ros::NodeHandle &nh, CarOnFloor floor = CarOnFloor::E0): caronfloor(floor){
    getWinddowParamter(nh);
  }
  void getWinddowParamter(ros::NodeHandle &nh){
    /* get Parameter about the Windows (e.g. where the map changed ) from the config file 
   * So we can change the window area without compilation.
   */
    nh.getParam("E0_E1_x1",E0_E1_x1);
    nh.getParam("E0_E1_x2",E0_E1_x2);
    nh.getParam("E0_E1_y1",E0_E1_y1);
    nh.getParam("E0_E1_y2",E0_E1_y2);
    //show_window_in_terminal(E0_E1_x1,E0_E1_x2,E0_E1_y1,E0_E1_y2,"E0_E1_Fenster");

    nh.getParam("E1_R10_x1",E1_R10_x1);
    nh.getParam("E1_R10_x2",E1_R10_x2);
    nh.getParam("E1_R10_y1",E1_R10_y1);
    nh.getParam("E1_R10_y2",E1_R10_y2);
    //show_window_in_terminal(E1_R10_x1,E1_R10_x2,E1_R10_y1,E1_R10_y2,"E1_R10_Fenster");

    nh.getParam("R10_E0_x1",R10_E0_x1);
    nh.getParam("R10_E0_x2",R10_E0_x2);
    nh.getParam("R10_E0_y1",R10_E0_y1);
    nh.getParam("R10_E0_y2",R10_E0_y2);
    //show_window_in_terminal(R10_E0_x1,R10_E0_x2,R10_E0_y1,R10_E0_y2,"R10_E0_Fenster");

    nh.getParam("E1_R12_x1",E1_R12_x1);
    nh.getParam("E1_R12_x2",E1_R12_x2);
    nh.getParam("E1_R12_y1",E1_R12_y1);
    nh.getParam("E1_R12_y2",E1_R12_y2);
    //show_window_in_terminal(E1_R12_x1,E1_R12_x2,E1_R12_y1,E1_R12_y2,"E1_R12_Fenster");

    nh.getParam("R12_E2_x1",R12_E2_x1);
    nh.getParam("R12_E2_x2",R12_E2_x2);
    nh.getParam("R12_E2_y1",R12_E2_y1);
    nh.getParam("R12_E2_y2",R12_E2_y2);
    //show_window_in_terminal(R12_E2_x1,R12_E2_x2,R12_E2_y1,R12_E2_y2,"R12_E2_Fenster");

    nh.getParam("E2_R21_x1",E2_R21_x1);
    nh.getParam("E2_R21_x2",E2_R21_x2);
    nh.getParam("E2_R21_y1",E2_R21_y1);
    nh.getParam("E2_R21_y2",E2_R21_y2);
    //show_window_in_terminal(E2_R21_x1,E2_R21_x2,E2_R21_y1,E2_R21_y2,"E2_R21_Fenster");

    nh.getParam("R21_E1_x1",R21_E1_x1);
    nh.getParam("R21_E1_x2",R21_E1_x2);
    nh.getParam("R21_E1_y1",R21_E1_y1);
    nh.getParam("R21_E1_y2",R21_E1_y2);
    //show_window_in_terminal(R21_E1_x1,R21_E1_x2,R21_E1_y1,R21_E1_y2,"R21_E1_Fenster");
  }


  void show_window_in_terminal(int x1, int x2, int y1, int y2,std::string map_name){
    std::cout << std::endl;
    std::cout << map_name << std::endl;
    std::cout << "   ---------- "<< std::endl;
    std::cout << y2<<"|" << "          " <<"|" << std::endl;
    std::cout << "  |" << "          " <<"|" << std::endl;
    std::cout << "  |" << "          " <<"|" << std::endl;
    std::cout << "  |" << "          " <<"|" << std::endl;
    std::cout << "  |" << "          " <<"|" << std::endl;
    std::cout << y1 << "|" << "          " <<"|" << std::endl;
    std::cout << "   ---------- "<< std::endl;
    std::cout <<"  " <<x1 << "        "<< x2 << std::endl;
  }

  void printFloor(){
    if(caronfloor == CarOnFloor::E0){
      std::cout << "\n  ---------Auto ist auf Ebene 0---------  " << std::endl; 
    }

    else if(caronfloor == CarOnFloor::E1){
      std::cout << "\n  ---------Auto ist auf Ebene 1---------  " << std::endl; 
    }

    else if(caronfloor == CarOnFloor::E2){
      std::cout << "\n  ---------Auto ist auf Ebene 2---------  " << std::endl; 
    }

    else if(caronfloor == CarOnFloor::R12){
      std::cout << "\n  ---------Auto ist an der Rampe R12---------  " << std::endl; 
    }

    else if(caronfloor == CarOnFloor::R10){
      std::cout << "\n  ---------Auto ist an der Rampe R10---------  " << std::endl; 
    }

    else if(caronfloor == CarOnFloor::R21){
      std::cout << "\n  ---------Auto ist an der Rampe R21---------  " << std::endl; 
    }
  }

  void check_floor(const State &state){
    if(caronfloor == CarOnFloor::E0){
      // Aufo kann nur auf Ebene 1 aufsteigen oder auf Ebne 0 bleiben
      if ( (state.x > E0_E1_x1 ) && (state.x < E0_E1_x2 ) && (state.y > E0_E1_y1 ) && (state.y < E0_E1_y2 ) ){
          caronfloor = CarOnFloor::E1;
          printFloor();
        }
    }
    else if(caronfloor == CarOnFloor::E1){
      if( (state.x > E1_R10_x1 ) && (state.x < E1_R10_x2 ) && (state.y > E1_R10_y1 ) && (state.y < E1_R10_y2 )){
            caronfloor = CarOnFloor::R10;
            printFloor();
            std::cout << "\n change map from map 1 RO_E1 to map 2 E1_E0\n";
            my_map.readRawData(map_pc, map_carpark,"src/icp_lokalisierung/scan_matching/map/map_Elphi_E1_E0_Bauplan_reduced.csv");
            map_carpark_change = true;
        }

      else if( (state.x > E1_R12_x1 ) && (state.x < E1_R12_x2 ) && (state.y > E1_R12_y1 ) && (state.y < E1_R12_y2 )){
            caronfloor = CarOnFloor::R12;
            printFloor();
            std::cout << "\n change map from map 1 RO_E1 to map 3 R12\n";
            my_map.readRawData(map_pc, map_carpark,"src/icp_lokalisierung/scan_matching/map/map_Elphi_R12_Bauplan_reduced.csv");
            map_carpark_change = true;
        }
    }
    else if(caronfloor == CarOnFloor::R10){
      if ( (state.x > R10_E0_x1 ) && (state.x < R10_E0_x2 ) && (state.y > R10_E0_y1 ) && (state.y < R10_E0_y2 )){
          caronfloor = CarOnFloor::E0;
          printFloor();
        }
    }
    else if(caronfloor == CarOnFloor::R12){
      if ( (state.x > R12_E2_x1 ) && (state.x < R12_E2_x2 ) && (state.y > R12_E2_y1 ) && (state.y < R12_E2_y2 ) )
        {
          caronfloor = CarOnFloor::E2;
          printFloor();
          std::cout << "\n change map from map 3 R12 to ma4 3 E2\n";
          my_map.readRawData(map_pc, map_carpark,"src/icp_lokalisierung/scan_matching/map/map_Elphi_E2_Bauplan_reduced.csv");    
          map_carpark_change = true;
        }
    }
    else if(caronfloor == CarOnFloor::E2){
      if ( (state.x > E2_R21_x1 ) && (state.x < E2_R21_x2 ) && (state.y > E2_R21_y1 ) && (state.y < E2_R21_y2 )){
          caronfloor = CarOnFloor::R21;
          printFloor();
          std::cout << "\n change map from map 4 E2 to map 5 R21\n";     
          my_map.readRawData(map_pc, map_carpark,"src/icp_lokalisierung/scan_matching/map/map_Elphi_R21.csv"); 
          map_carpark_change = true;
        }
    }
    else if(caronfloor == CarOnFloor::R21){
      if (
          (state.x > R21_E1_x1 ) &&
          (state.x < R21_E1_x2 ) &&
          (state.y > R21_E1_y1 ) &&
          (state.y < R21_E1_y2 )
        )
        {
          caronfloor = CarOnFloor::E1;
          printFloor();
          std::cout << "\n change map from map 5 R21 to map 1 R0_R1\n";
          my_map.readRawData(map_pc, map_carpark,"src/icp_lokalisierung/scan_matching/map/map_Elphi_R0_E1_Bauplan_reduced.csv");
          map_carpark_change = true;
        }
    }


  }


 private:
  // E0 -> E1 Fenster
  double E0_E1_x1;
  double E0_E1_x2;
  double E0_E1_y1;
  double E0_E1_y2;
  // E1 -> R10 Fenster
  double E1_R10_x1;
  double E1_R10_x2;
  double E1_R10_y1;
  double E1_R10_y2;
  // R10 -> E0 Fenster
  double R10_E0_x1;
  double R10_E0_x2;
  double R10_E0_y1;
  double R10_E0_y2;
  // E1 -> R12 Fenster
  double E1_R12_x1;
  double E1_R12_x2;
  double E1_R12_y1;
  double E1_R12_y2;
  // R12 -> E2 Fenster
  double R12_E2_x1;
  double R12_E2_x2;
  double R12_E2_y1;
  double R12_E2_y2;
  // E2 -> R21 Fenster
  double E2_R21_x1; 
  double E2_R21_x2; 
  double E2_R21_y1; 
  double E2_R21_y2; 
  // R21 -> E1 Fenster
  double R21_E1_x1;
  double R21_E1_x2;
  double R21_E1_y1;
  double R21_E1_y2;


};
#endif