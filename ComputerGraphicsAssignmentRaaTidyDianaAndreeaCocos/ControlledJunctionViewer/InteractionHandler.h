#pragma once

#include <windows.h>
#include <osgGA/GUIEventHandler>

typedef void (KEY_FUNC)(char c);

class InteractionHandler : public osgGA::GUIEventHandler
{
public:
	InteractionHandler(KEY_FUNC* pFunc = 0);
	virtual ~InteractionHandler();
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*);

protected:
	KEY_FUNC* m_pFunc;

};

