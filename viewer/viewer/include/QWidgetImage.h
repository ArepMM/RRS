/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2009 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

#ifndef QWIDGETIMAGE
#define QWIDGETIMAGE

#include "QGraphicsViewAdapter.h"
#include <osg/Image>

namespace osgQt
{

class QWidgetImage : public osg::Image
{
    public:

        QWidgetImage( QWidget* widget=0 );

        QWidget* getQWidget() { return _widget; }
        QGraphicsViewAdapter* getQGraphicsViewAdapter() { return _adapter; }

        virtual bool requiresUpdateCall() const { return true; }
        virtual void update( osg::NodeVisitor* /*nv*/ ) { render(); }

        void clearWriteBuffer();

        void render();

        /// Overridden scaleImage used to catch cases where the image is
        /// fullscreen and the window is resized.
        virtual void scaleImage(int s,int t,int r, GLenum newDataType);

        virtual bool sendFocusHint(bool focus);

        virtual bool sendPointerEvent(int x, int y, int buttonMask);

        virtual bool sendKeyEvent(int key, bool keyDown);

        virtual void setFrameLastRendered(const osg::FrameStamp* frameStamp);

    protected:

        QPointer<QGraphicsViewAdapter>  _adapter;
        QPointer<QWidget>               _widget;
};

}

#endif
