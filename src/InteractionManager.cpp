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
  const unsigned long REPEAT_DELAY = 80; // 增加长按重复触发延迟，从60ms改为80ms，否则容易误触
  
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
      
      // 缩放由 main.cpp 中的 handleControls 统一进行 2D/3D 平滑连续缩放处理
      // 此处专注于检测空格键、t键以及方向键平移
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
