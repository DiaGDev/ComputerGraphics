#include "ShaderAttacher.h"

#include <iostream>

#include "ShaderUnit.h"

#include <osg/Geode>

Shader::ShaderAttacher::ShaderAttacher(osg::Node* pRoot) :osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), m_sName(""), m_pCamera(0)
{
	traverse(*pRoot);
}

Shader::ShaderAttacher::ShaderAttacher(osg::Node* pRoot, std::string sShaderName, osg::Camera* pCamera): osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), m_sName(sShaderName), m_pCamera(pCamera)
{
	traverse(*pRoot);
}

Shader::ShaderAttacher::~ShaderAttacher()
{
}

void Shader::ShaderAttacher::apply(osg::Node& rNode)
{
	std::string s;
	for (int i = 0; i < getNodePath().size(); i++) s += "|--";
	std::cout << s << rNode.className() <<std::endl;

	if(osg::Geode* pGeode=dynamic_cast<osg::Geode*>(&rNode))
	{
		if (m_pCamera)
			ShaderUnit::addShaderToGeode(m_sName, pGeode, m_pCamera);
		else
			pGeode->getOrCreateStateSet()->setAttributeAndModes((osg::Program*)0, osg::StateAttribute::OFF);
	}
	traverse(rNode);
}
