#pragma once

#include <windows.h>
#include <osgGA/GUIEventHandler>

typedef void (KEY_FUNC)(char c);

class ExampleInteractionHandler : public osgGA::GUIEventHandler
{
public:
	ExampleInteractionHandler(KEY_FUNC* pFunc=0);
	virtual ~ExampleInteractionHandler();
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*);

protected:
	KEY_FUNC* m_pFunc;

};

