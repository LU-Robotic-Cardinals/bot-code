#include "vex.h"
#include <cmath>
#include <string>
#include <vector>
// #include <algorithm>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>

std::string hex_to_string(unsigned int color) {
    std::stringstream ss;
    ss << "#" << std::uppercase << std::hex << std::setw(6) << std::setfill('0') << color;
    return ss.str();
}

struct Box {
  int tl_x, tl_y, br_x, br_y;
  int border_thickness;
  unsigned int fill_color, outline_color;
  // int selected_portal
};

class GenerateBoxes{
  private:
    int max_screen_x = 480;
    int max_screen_y = 240;
    int box_gap = 5;
    int border = 3;
  public:

  std::vector<Box> generate(int num_x,int num_y){
    std::vector<Box> box_list;
    for (int i = 0; i < num_x; i++){
    for (int j = 0; j < num_y; j++){
      int box_color = 0xffffff;
      int box_size_x = (max_screen_x - box_gap - (box_gap * num_x)) / num_x;
      int box_size_y = (max_screen_y - box_gap - (box_gap * num_y)) / num_y;

      Box new_box = {
        // box_size_x,
        box_size_x * i + (box_gap * (i + 1)),
        box_size_y * j + (box_gap * (j + 1)),
        box_size_x * (i+1) + (box_gap * (i + 1)),
        box_size_y * (j+1) + (box_gap * (j + 1)),
        border,
        box_color
      };
      box_list.push_back(new_box);

    }}
    return box_list;
  }
};

class ShowBoxes{
  private:
  public:
  void show(brain brain_obj, std::vector<Box> boxes){
    brain_obj.Screen.clearScreen();
    for (int i = 0; i < boxes.size(); i++) {
      Box c_box = boxes[i];
      
      // Border color
      std::string color = hex_to_string(c_box.outline_color);
      // brain_obj.Screen.setFillColor(color);
      // brain_obj.Screen.setPenColor(color);
      brain_obj.Screen.setFillColor(hex_to_string(c_box.outline_color));
      brain_obj.Screen.setPenColor(hex_to_string(c_box.outline_color));

      brain_obj.Screen.drawRectangle(
        c_box.tl_x,  c_box.tl_y,
        c_box.br_x-c_box.tl_x,
        c_box.br_y-c_box.tl_y);
      
      // Inside of the box
      brain_obj.Screen.setFillColor(hex_to_string(c_box.fill_color));
      brain_obj.Screen.setPenColor(hex_to_string(c_box.fill_color));
      brain_obj.Screen.drawRectangle(
        c_box.tl_x+c_box.border_thickness,  c_box.tl_y+c_box.border_thickness,
        c_box.br_x-c_box.tl_x - 2 * c_box.border_thickness,
        c_box.br_y-c_box.tl_y - 2 * c_box.border_thickness);
    }
  }
};