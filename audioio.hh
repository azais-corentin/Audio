#pragma once

#include <QObject>

class AudioIO {
 public:
  AudioIO();
  ~AudioIO();

 private:
  bool mInitSuccessful = false;
};
