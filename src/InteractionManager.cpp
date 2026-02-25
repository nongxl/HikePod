#include "InteractionManager.h"
#include <M5Cardputer.h>
#include <algorithm>

  InteractionManager::InteractionManager() : 
  zoomLevel(8),  // 默认使用 ZOOM_5KM
  targetZoomLevel(8),  // 默认使用 ZOOM_5KM
  panOffsetX(0),
  panOffsetY(0),
  targetPanOffsetX(0),
  targetPanOffsetY(0),
  zoomChanged(false),
  panChanged(false),
  isPanAnimating(false),
  isZoomAnimating(false),
  spaceKeyPressed(false),
  tKeyPressed(false) {
}

void InteractionManager::begin() {
}

void InteractionManager::update(bool keyboardChanged, bool keyboardPressed, Keyboard_Class::KeysState keys) {
  handleKeyboardInput(keyboardChanged, keyboardPressed, keys);
  updateAnimation();
}

void InteractionManager::updateAnimation() {
  bool needsUpdate = false;
  
  // 处理平移动画
  if (isPanAnimating) {
    int dx = targetPanOffsetX - panOffsetX;
    int dy = targetPanOffsetY - panOffsetY;
    
    if (abs(dx) >= 1 || abs(dy) >= 1) {
      panOffsetX += dx * 0.5; // 增加缓动系数，从0.2改为0.5
      panOffsetY += dy * 0.5; // 增加缓动系数，从0.2改为0.5
      panChanged = true;
      needsUpdate = true;
    } else {
      panOffsetX = targetPanOffsetX;
      panOffsetY = targetPanOffsetY;
      isPanAnimating = false;
    }
  }
  
  // 处理缩放动画
  if (isZoomAnimating) {
    int dz = targetZoomLevel - zoomLevel;
    if (abs(dz) > 0) {
      // 逐步接近目标值，每次变化1
      zoomLevel += (dz > 0) ? 1 : -1;
      zoomChanged = true;
      needsUpdate = true;
      
      // 只有当达到目标值时才结束动画
      if (zoomLevel == targetZoomLevel) {
        isZoomAnimating = false;
      }
    }
  }
}

int InteractionManager::getZoomLevel() {
  return targetZoomLevel;
}

void InteractionManager::getPanOffset(int& x, int& y) {
  x = panOffsetX;
  y = panOffsetY;
}

void InteractionManager::setPanOffset(int x, int y) {
  panOffsetX = x;
  panOffsetY = y;
  targetPanOffsetX = x;
  targetPanOffsetY = y;
  // 不再重置targetZoomLevel，避免缩放级别跳级
  isPanAnimating = false;
}

void InteractionManager::setZoomLevel(int level) {
  zoomLevel = level;
  targetZoomLevel = level;
  isZoomAnimating = false;
}

bool InteractionManager::isZoomChanged() {
  return zoomChanged;
}

bool InteractionManager::isPanChanged() {
  return panChanged;
}

void InteractionManager::handleKeyboardInput(bool keyboardChanged, bool keyboardPressed, Keyboard_Class::KeysState keys) {
  static unsigned long lastKeyPressTime = 0;
  static unsigned long lastRepeatTime = 0;
  const unsigned long DEBOUNCE_DELAY = 100; // 保持防抖延迟
  const unsigned long REPEAT_DELAY = 60; // 减少长按重复触发延迟，从100ms改为60ms
  
  // 每次调用先重置标志
  zoomChanged = false;
  panChanged = false;
  
  // 检测键盘状态变化或按键按下
  if (keyboardChanged || keyboardPressed) {
    unsigned long currentTime = millis();
    
    // 检测初始按下或长按重复
    bool shouldProcess = false;
    if (keyboardChanged && keyboardPressed) {
      // 初始按下
      if (currentTime - lastKeyPressTime >= DEBOUNCE_DELAY) {
        shouldProcess = true;
        lastKeyPressTime = currentTime;
        lastRepeatTime = currentTime + REPEAT_DELAY; // 设置长按重复开始时间
      }
    } else if (keyboardPressed) {
      // 长按重复
      if (currentTime >= lastRepeatTime) {
        shouldProcess = true;
        lastRepeatTime = currentTime + REPEAT_DELAY; // 更新下次重复时间
      }
    }
    
    if (shouldProcess) {
      // 检测是否是初始按下
      bool isInitialPress = (keyboardChanged && keyboardPressed);
      
      // 检测加号键（包括Shift+加号的情况）
      bool plusPressed = false;
      for (auto key : keys.word) {
        if (key == '+' || key == '=') {  // 有些键盘上+和=是同一个键
          plusPressed = true;
          break;  // 找到一个就停止，避免重复触发
        }
      }
      
      // 检测减号键（包括Shift+-的_情况）
      bool minusPressed = false;
      if (!plusPressed) { // 如果已经检测到加号键，就不再检测减号键
        for (auto key : keys.word) {
          if (key == '-' || key == '_') {  // - 和 _ 都作为缩小
            minusPressed = true;
            break;  // 找到一个就停止，避免重复触发
          }
        }
      }
      
      // 处理缩放操作
      if (plusPressed) {
        if (targetZoomLevel > 0) {  // 支持最多11个缩放级别（0-10）
          targetZoomLevel--;  // 减小缩放级别 = 放大地图
          zoomChanged = true;
          isZoomAnimating = true;
          //Serial.printf("[DEBUG] Plus/Equal key pressed, targetZoomLevel=%d\n", targetZoomLevel);
        }
      } else if (minusPressed) {
        if (targetZoomLevel < 10) {
          targetZoomLevel++;  // 增加缩放级别 = 缩小地图
          zoomChanged = true;
          isZoomAnimating = true;
          //Serial.printf("[DEBUG] Minus key pressed, targetZoomLevel=%d\n", targetZoomLevel);
        }
      } else {
        // 检测空格键
        bool spaceFound = false;
        bool tKeyFound = false;
        for (auto key : keys.word) {
          if (key == ' ') {
            spaceFound = true;
          } else if (key == 't') {
            tKeyFound = true;
          }
        }
        if (spaceFound) {
          spaceKeyPressed = true;
          Serial.println("[DEBUG] Space key pressed");
        } else if (tKeyFound) {
          tKeyPressed = true;
          Serial.println("[DEBUG] T key pressed");
        } else {
          // 只有在没有检测到缩放键、空格键和t键时，才处理方向键
          // 其他方向控制 - 直接使用keys.word来检测按键
          for (auto key : keys.word) {
            int panDistance = isInitialPress ? 15 : 6; // 单次按键平移距离为15，长按为6（增加移动距离）
            
            if (key == ';') {
              targetPanOffsetY += panDistance; // 上键：分号 - 反转：向上移动内容
              isPanAnimating = true;
              panChanged = true;
              static bool panUpLogged = false;
              if (!panUpLogged) {
                Serial.println("[PAN DEBUG] Pan up");
                panUpLogged = true;
              }
            } else if (key == '.') {
              targetPanOffsetY -= panDistance; // 下键：句号 - 反转：向下移动内容
              isPanAnimating = true;
              panChanged = true;
              static bool panDownLogged = false;
              if (!panDownLogged) {
                Serial.println("[PAN DEBUG] Pan down");
                panDownLogged = true;
              }
            } else if (key == ',') {
              targetPanOffsetX += panDistance; // 左键：逗号 - 反转：向左移动内容
              isPanAnimating = true;
              panChanged = true;
              static bool panLeftLogged = false;
              if (!panLeftLogged) {
                Serial.println("[PAN DEBUG] Pan left");
                panLeftLogged = true;
              }
            } else if (key == '/') {
              targetPanOffsetX -= panDistance; // 右键：斜杠 - 反转：向右移动内容
              isPanAnimating = true;
              panChanged = true;
              static bool panRightLogged = false;
              if (!panRightLogged) {
                Serial.println("[PAN DEBUG] Pan right");
                panRightLogged = true;
              }
            }
          }
        }
      }
    }
  }
}

bool InteractionManager::isSpaceKeyPressed() {
  return spaceKeyPressed;
}

void InteractionManager::resetSpaceKeyPressed() {
  spaceKeyPressed = false;
}

bool InteractionManager::isTKeyPressed() {
  return tKeyPressed;
}

void InteractionManager::resetTKeyPressed() {
  tKeyPressed = false;
}
