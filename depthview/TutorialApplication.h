/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.h
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#ifndef __TutorialApplication_h_
#define __TutorialApplication_h_

#include "BaseApplication.h"

#include <libfreenect.h>
#include <libfreenect_sync.h>
#include <math.h>

#define PI 3.14159265
#define MAX_VERTEX_COUNT 307200 // 640*480
#define MAX_TRIANGLE_COUNT 612162 //(640-1)*(480-1)*2
#define MAX_INDECES_COUNT 1836486 // MAX_TRIANGLE_COUNT*3

class TutorialApplication : public BaseApplication
{
public:
    TutorialApplication(void);
    virtual ~TutorialApplication(void);

    Ogre::RenderWindow * getWindow(void) { return mWindow; }
    Ogre::Timer * getTimer(void) { return mTimer; }
    OIS::Mouse * getMouse(void) { return mMouse; }
    OIS::Keyboard * getKeyboard(void) { return mKeyboard; }
protected:
    virtual void createScene(void);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
    Ogre::Timer *mTimer;

    Ogre::HardwareVertexBufferSharedPtr vertexBuffer;
    Ogre::HardwareIndexBufferSharedPtr indexBuffer;
    bool keyPressed( const OIS::KeyEvent &arg );

    void loadCalibration();
    void resetCalibration();

private:
    // Значения, которые появлялись в процессе работы и их количество
    std::map< int, int > usedValues;
    // Карта глубины
    unsigned short *depth;
    // Матрица с данными калибровки
    float calibration[MAX_VERTEX_COUNT];

    // Матрица глубин. Хранит данные после калибровки
    float depthMatrix[MAX_VERTEX_COUNT];

    float rawDepthToMM(float depth_value);
    void virtualPointToReal(int x, int y, float rawDepth, float* point);

    // Создаваемая вручную сетка и ее элементы
    Ogre::MeshPtr mesh;
    Ogre::SubMesh *subMesh;

    /**
     * После получения глубин мы должны подготовить новый набор индексов.
     * Важно выполнить обработку на
    void splitMesh() {
    }*/
};

#endif // #ifndef __TutorialApplication_h_
