#pragma once

#include <windows.h>
#include <osg/NodeVisitor>

namespace Shader
{
	class ShaderAttacher: public osg::NodeVisitor
	{
	public:
		ShaderAttacher(osg::Node* pRoot);
		ShaderAttacher(osg::Node* pRoot, std::string sShaderName, osg::Camera* pCamera);
		virtual ~ShaderAttacher();
	protected:
		virtual void apply(osg::Node& rNode);
	private:
		std::string m_sName;
		osg::Camera* m_pCamera;

	};
}
