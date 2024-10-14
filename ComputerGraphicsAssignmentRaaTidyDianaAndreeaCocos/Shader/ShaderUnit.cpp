#include <windows.h>
#include <osg/Program>
#include <fstream>
#include <sstream>
#include <filesystem>
#include "ShaderUnit.h"

#include <iostream>

std::string Shader::ShaderUnit::sm_sShaderPath = "./";
Shader::ShaderUnits Shader::ShaderUnit::sm_mShaderUnits;
osg::Uniform* Shader::ShaderUnit::sm_pModelViewProjectionMatrixUniform=0;
osg::Uniform* Shader::ShaderUnit::sm_pCameraEyeUniform=0;

Shader::ShaderUnit::ShaderUnit(std::string sName): m_sName(sName), m_pProgram(0), m_vbStateControl({true, true, true, true}), m_vbStateControl2({ true, true, true, true }), m_pStateControlUniform(new osg::Uniform(osg::Uniform::BOOL_VEC4, "StateControl")), m_pStateControlUniform2(new osg::Uniform(osg::Uniform::BOOL_VEC4, "StateControl2"))
{
	m_pStateControlUniform->ref();
	m_pStateControlUniform->set(m_vbStateControl[0], m_vbStateControl[1], m_vbStateControl[2], m_vbStateControl[3]);
	m_pStateControlUniform2->ref();
	m_pStateControlUniform2->set(m_vbStateControl2[0], m_vbStateControl2[1], m_vbStateControl2[2], m_vbStateControl2[3]);
	loadShaders();
}

Shader::ShaderUnit::~ShaderUnit()
{
	for (Shaders::iterator it = m_mShaders.begin(); it != m_mShaders.end(); it++) it->second->unref();
	m_mShaders.clear();

	if (m_pProgram) m_pProgram->unref();
	if (m_pStateControlUniform) m_pStateControlUniform->unref();
	if (m_pStateControlUniform2) m_pStateControlUniform2->unref();

}

std::string Shader::ShaderUnit::typeToExtension(osg::Shader::Type eType)
{
	switch(eType)
	{
	case osg::Shader::FRAGMENT:
		return ".frag";
	case osg::Shader::VERTEX:
		return ".vert";
	case osg::Shader::COMPUTE:
		return ".comp";
	case osg::Shader::GEOMETRY:
		return ".geom";
	}

	return "";
}

bool Shader::ShaderUnit::addShader(std::string sName)
{
	if(sm_mShaderUnits.find(sName)==sm_mShaderUnits.end())
	{
		sm_mShaderUnits[sName] = new ShaderUnit(sName);
		return true;
	}
	return false;
}

osg::Program* Shader::ShaderUnit::shaderProgram(std::string sName)
{
	if (sm_mShaderUnits.find(sName) != sm_mShaderUnits.end())
	{
		return sm_mShaderUnits[sName]->m_pProgram;
	}
	return 0;
}

void Shader::ShaderUnit::rebuildShader(std::string sName)
{
	if (sm_mShaderUnits.find(sName) != sm_mShaderUnits.end())
	{
		sm_mShaderUnits[sName]->loadShaders();
	}
}

void Shader::ShaderUnit::addShaderToGeode(std::string sName, osg::Geode* pGeode, osg::Camera* pCamera)
{
	if (sm_mShaderUnits.find(sName) != sm_mShaderUnits.end() && pGeode && pCamera)
	{
		osg::StateSet* pSS = pGeode->getOrCreateStateSet();
		pSS->setAttributeAndModes(sm_mShaderUnits[sName]->m_pProgram, osg::StateAttribute::ON);
		pSS->addUniform(sm_pModelViewProjectionMatrixUniform);
		pSS->addUniform(sm_pCameraEyeUniform);
		pSS->addUniform(sm_mShaderUnits[sName]->m_pStateControlUniform);
		pSS->addUniform(sm_mShaderUnits[sName]->m_pStateControlUniform2);



//		pSS->setMode(GL_TEXTURE, osg::StateAttribute::ON);
/*
		pSS->setMode(GL_LIGHTING, osg::StateAttribute::ON);
		pSS->setMode(GL_BLEND, osg::StateAttribute::ON);
		pSS->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);

		osg::BlendFunc* blendfunc = new osg::BlendFunc();
		state->setAttributeAndModes(blendfunc, osg::StateAttribute::ON);
*/
	}
}

void Shader::ShaderUnit::setStateControl(std::string sName, std::string sControlItem, bool bState)
{
	if (sm_mShaderUnits.find(sName) != sm_mShaderUnits.end())
	{
		ShaderUnit* pUnit = sm_mShaderUnits[sName];

		if (sControlItem == "ambient") pUnit->m_vbStateControl[0] = bState;
		else if (sControlItem == "diffuse") pUnit->m_vbStateControl[1] = bState;
		else if (sControlItem == "specular") pUnit->m_vbStateControl[2] = bState;
		else if (sControlItem == "texture") pUnit->m_vbStateControl[3] = bState;
		else if (sControlItem == "bump") pUnit->m_vbStateControl2[0] = bState;
		pUnit->m_pStateControlUniform->set(pUnit->m_vbStateControl[0], pUnit->m_vbStateControl[1], pUnit->m_vbStateControl[2], pUnit->m_vbStateControl[3]);
		pUnit->m_pStateControlUniform2->set(pUnit->m_vbStateControl2[0], pUnit->m_vbStateControl2[1], pUnit->m_vbStateControl2[2], pUnit->m_vbStateControl2[3]);
	}
}



void Shader::ShaderUnit::toggleStateControl(std::string sName, std::string sControlItem)
{
	if (sm_mShaderUnits.find(sName) != sm_mShaderUnits.end())
	{
		ShaderUnit* pUnit = sm_mShaderUnits[sName];

		if (sControlItem == "ambient") pUnit->m_vbStateControl[0] = !pUnit->m_vbStateControl[0];
		else if (sControlItem == "diffuse") pUnit->m_vbStateControl[1] = !pUnit->m_vbStateControl[1];
		else if (sControlItem == "specular") pUnit->m_vbStateControl[2] = !pUnit->m_vbStateControl[2];
		else if (sControlItem == "texture") pUnit->m_vbStateControl[3] = !pUnit->m_vbStateControl[3];
		else if (sControlItem == "bump") pUnit->m_vbStateControl2[0] = !pUnit->m_vbStateControl2[0];
		pUnit->m_pStateControlUniform->set(pUnit->m_vbStateControl[0], pUnit->m_vbStateControl[1], pUnit->m_vbStateControl[2], pUnit->m_vbStateControl[3]);
		pUnit->m_pStateControlUniform2->set(pUnit->m_vbStateControl2[0], pUnit->m_vbStateControl2[1], pUnit->m_vbStateControl2[2], pUnit->m_vbStateControl2[3]);
	}
}

void Shader::ShaderUnit::createCameraBasedUniformCallbacks(osg::Camera* pCamera)
{
	if (!sm_pModelViewProjectionMatrixUniform)
	{
		sm_pModelViewProjectionMatrixUniform = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "ModelViewProjectionMatrix");
		sm_pModelViewProjectionMatrixUniform->setUpdateCallback(new Shader::ShaderUnit::ModelViewProjectionMatrixCallback(pCamera));
	}

	if (!sm_pCameraEyeUniform)
	{
		sm_pCameraEyeUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "CameraEye");
		sm_pCameraEyeUniform->setUpdateCallback(new Shader::ShaderUnit::CameraEyeCallback(pCamera));
	}
}

void Shader::ShaderUnit::setShaderPath(std::string sPath)
{
	sm_sShaderPath = sPath;
}

void Shader::ShaderUnit::loadShaders()
{
	if (!m_pProgram)
	{
		m_pProgram = new osg::Program();
		m_pProgram->ref();
	}

	loadShader(osg::Shader::FRAGMENT);
	loadShader(osg::Shader::VERTEX);
	loadShader(osg::Shader::COMPUTE);
	loadShader(osg::Shader::GEOMETRY);
}

void Shader::ShaderUnit::loadShader(osg::Shader::Type eType)
{
	std::string sFile = sm_sShaderPath + "/" + m_sName + typeToExtension(eType);
	if (std::filesystem::exists(sFile))
	{
		if(m_mShaders.find(eType) == m_mShaders.end())
		{
			m_mShaders[eType] = new osg::Shader(eType);
			m_mShaders[eType]->ref();
			m_mShaders[eType]->loadShaderSourceFromFile(sFile);
			m_pProgram->addShader(m_mShaders[eType]);
		}
		else
		{
			m_mShaders[eType]->loadShaderSourceFromFile(sFile);
			m_mShaders[eType]->dirtyShader();
		}
	}
	else
	{
		if (m_mShaders.find(eType) != m_mShaders.end())
		{
			m_pProgram->removeShader(m_mShaders[eType]);
			m_mShaders[eType]->unref();
			m_mShaders.erase(eType);
		}
	}
}

Shader::ShaderUnit::ModelViewProjectionMatrixCallback::ModelViewProjectionMatrixCallback(osg::Camera* pCamera): m_pCamera(pCamera)
{
}

Shader::ShaderUnit::ModelViewProjectionMatrixCallback::~ModelViewProjectionMatrixCallback()
{
}

void Shader::ShaderUnit::ModelViewProjectionMatrixCallback::operator()(osg::Uniform* pUniform, osg::NodeVisitor* pNV)
{
	osg::Matrixd viewMatrix = m_pCamera->getViewMatrix();
	osg::Matrixd modelMatrix = osg::computeLocalToWorld(pNV->getNodePath());
	osg::Matrixd modelViewProjectionMatrix = modelMatrix * viewMatrix * m_pCamera->getProjectionMatrix();
	pUniform->set(modelViewProjectionMatrix);
}

Shader::ShaderUnit::CameraEyeCallback::CameraEyeCallback(osg::Camera* pCamera): m_pCamera(pCamera)
{
}

Shader::ShaderUnit::CameraEyeCallback::~CameraEyeCallback()
{
}

void Shader::ShaderUnit::CameraEyeCallback::operator()(osg::Uniform* pUniform, osg::NodeVisitor* pNV)
{
	osg::Vec3f eye, center, up;
	m_pCamera->getViewMatrixAsLookAt(eye, center, up);
	osg::Vec4f eye_vec = osg::Vec4f(eye.x(), eye.y(), eye.z(), 1);
	pUniform->set(eye_vec);
}
