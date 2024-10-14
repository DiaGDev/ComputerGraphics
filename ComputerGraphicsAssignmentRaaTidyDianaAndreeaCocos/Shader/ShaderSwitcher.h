#pragma once

#include <windows.h>
#include <osgGA/GUIEventHandler>

namespace Shader
{
	class ShaderAttacher;

	class ShaderSwitcher : public osgGA::GUIEventHandler
	{
	public:
		ShaderSwitcher(osg::Node* pRoot);
		virtual ~ShaderSwitcher();

		virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*);
	protected:
		ShaderAttacher* m_pAttacher;
		osg::Node* m_pRoot;
		std::string m_sCurrentShader;
	};
}
