/*
 -----------------------------------------------------------------------------
 This source file is part of OGRE
 (Object-oriented Graphics Rendering Engine)
 For the latest info, see http://www.ogre3d.org/

 Copyright (c) 2000-2014 Torus Knot Software Ltd

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 -----------------------------------------------------------------------------
 */
// File modified to change OIS to Qt KeyEvents
#ifndef TESSERACT_VISUALIZATION_QT_OGRE_WINDOW_H
#define TESSERACT_VISUALIZATION_QT_OGRE_WINDOW_H

/* Qt headers */
#include <QtWidgets/QApplication>
#include <QtGui/QKeyEvent>
#include <QtGui/QWindow>

/* Ogre3D header */
#ifndef Q_MOC_RUN
#include <Ogre.h>
#endif

/* Changed SdkCameraMan implementation to work with QKeyEvent, QMouseEvent, QWheelEvent */
#include "tesseract_visualization/core/qt_ogre_camera_man.h"

namespace tesseract_visualization
{
/*
With the headers included we now need to inherit from QWindow.
*/
class QTOgreWindow : public QWindow, public Ogre::FrameListener
{
  /*
  A QWindow still inherits from QObject and can have signals/slots; we need to add the appropriate
  Q_OBJECT keyword so that Qt's intermediate compiler can do the necessary wireup between our class
  and the rest of Qt.
  */
  Q_OBJECT

public:
  explicit QTOgreWindow(QWindow *parent = NULL);
  ~QTOgreWindow();

  /*
  We declare these methods virtual to allow for further inheritance.
  */
  virtual void render(QPainter *painter);
  virtual void render();
  virtual void initialize();
  virtual void createScene();
#if OGRE_VERSION >= ((2 << 16) | (0 << 8) | 0)
  virtual void createCompositor();
#endif

  void setAnimating(bool animating);

public Q_SLOTS:

  virtual void renderLater();
  virtual void renderNow();

  /*
  We use an event filter to be able to capture keyboard/mouse events. More on this later.
  */
  virtual bool eventFilter(QObject *target, QEvent *event);

Q_SIGNALS:
  /*
  Event for clicking on an entity.
  */
  void entitySelected(Ogre::Entity* entity);

protected:
  /*
  Ogre3D pointers added here. Useful to have the pointers here for use by the window later.
  */
  Ogre::Root* m_ogreRoot;
  Ogre::RenderWindow* m_ogreWindow;
  Ogre::SceneManager* m_ogreSceneMgr;
  Ogre::Camera* m_ogreCamera;
  Ogre::ColourValue m_ogreBackground;
  OgreQtBites::SdkQtCameraMan* m_cameraMan;

  bool m_update_pending;
  bool m_animating;

  /*
  The below methods are what is actually fired when they keys on the keyboard are hit.
  Similar events are fired when the mouse is pressed or other events occur.
  */
  virtual void keyPressEvent(QKeyEvent * ev);
  virtual void keyReleaseEvent(QKeyEvent * ev);
  virtual void mouseMoveEvent(QMouseEvent* e);
  virtual void wheelEvent(QWheelEvent* e);
  virtual void mousePressEvent(QMouseEvent* e);
  virtual void mouseReleaseEvent(QMouseEvent* e);
  virtual void exposeEvent(QExposeEvent *event);
  virtual bool event(QEvent *event);

  /*
  FrameListener method
  */
  virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

  /*
  Write log messages to Ogre log
  */
  void log(Ogre::String msg);
  void log(QString msg);
};
}
#endif // TESSERACT_VISUALIZATION_QT_OGRE_WINDOW_H
