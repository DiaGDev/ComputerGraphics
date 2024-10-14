#pragma once
#include <windows.h>
#include <string>
#include <map>

#include <osg/Program>
#include <osg/Shader>
#include <osg/Geode>

namespace Shader
{
	typedef std::map<osg::Shader::Type, osg::Shader*> Shaders;
	typedef std::map<std::string, class ShaderUnit*> ShaderUnits;

	class ShaderUnit
	{
	public:

		static void setShaderPath(std::string sPath);

		static bool addShader(std::string sName);
		static osg::Program* shaderProgram(std::string sName);
		static void rebuildShader(std::string sName);
		static void addShaderToGeode(std::string sName, osg::Geode* pGeode, osg::Camera* pCamera);

		static void setStateControl(std::string sName, std::string sControlItem, bool bState);
		static void toggleStateControl(std::string sName, std::string sControlItem);
		static void createCameraBasedUniformCallbacks(osg::Camera* pCamera);


		class ModelViewProjectionMatrixCallback: public osg::Uniform::Callback
		{
		public:
			ModelViewProjectionMatrixCallback(osg::Camera* pCamera);
			virtual ~ModelViewProjectionMatrixCallback();

			virtual void operator()(osg::Uniform* pUniform, osg::NodeVisitor* pNV);

		protected:
			osg::Camera* m_pCamera;
		};

		class CameraEyeCallback: public osg::Uniform::Callback
		{
		public:
			CameraEyeCallback(osg::Camera* pCamera);
			virtual ~CameraEyeCallback();

			virtual void operator()(osg::Uniform* pUniform, osg::NodeVisitor* pNV);

		protected:
			osg::Camera* m_pCamera;

		};


	protected:
		static std::string sm_sShaderPath;

		std::string m_sName;
		Shaders m_mShaders;

		void loadShaders();
		void loadShader(osg::Shader::Type eType);

		static osg::Uniform* sm_pModelViewProjectionMatrixUniform;
		static osg::Uniform* sm_pCameraEyeUniform;
		osg::Uniform* m_pStateControlUniform;
		osg::Uniform* m_pStateControlUniform2;

	private:
		osg::Program* m_pProgram;
		ShaderUnit(std::string sName);
		virtual ~ShaderUnit();

		static ShaderUnits sm_mShaderUnits;
		osg::Vec4b m_vbStateControl;
		osg::Vec4b m_vbStateControl2;

		static std::string typeToExtension(osg::Shader::Type eType);
	};
}

