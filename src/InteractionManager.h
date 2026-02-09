#ifndef INTERACTION_MANAGER_H
#define INTERACTION_MANAGER_H

#include <M5Cardputer.h>

class InteractionManager {
public:
  InteractionManager();
  
  // 初始化交互管理器
  void begin();
  
  // 更新交互状态
  void update(bool keyboardChanged, bool keyboardPressed, Keyboard_Class::KeysState keys);
  
  // 获取缩放级别
  int getZoomLevel();
  
  // 获取平移偏移
  void getPanOffset(int& x, int& y);
  
  // 设置平移偏移
  void setPanOffset(int x, int y);
  
  // 设置缩放级别
  void setZoomLevel(int level);
  
  // 检查缩放是否改变
  bool isZoomChanged();
  
  // 检查平移是否改变
  bool isPanChanged();
  
  // 检查空格键是否被按下
  bool isSpaceKeyPressed();
  
  // 重置空格键标志
  void resetSpaceKeyPressed();
  
  // 检查t键是否被按下
  bool isTKeyPressed();
  
  // 重置t键标志
  void resetTKeyPressed();
  
private:
  int zoomLevel;
  int targetZoomLevel;
  int panOffsetX;
  int panOffsetY;
  int targetPanOffsetX;
  int targetPanOffsetY;
  bool zoomChanged;
  bool panChanged;
  bool isPanAnimating;
  bool isZoomAnimating;
  bool spaceKeyPressed;  // 空格键是否被按下
  bool tKeyPressed;  // t键是否被按下
  
  // 处理键盘输入
  void handleKeyboardInput(bool keyboardChanged, bool keyboardPressed, Keyboard_Class::KeysState keys);
  
  // 更新动画
  void updateAnimation();
};

#endif // INTERACTION_MANAGER_H
