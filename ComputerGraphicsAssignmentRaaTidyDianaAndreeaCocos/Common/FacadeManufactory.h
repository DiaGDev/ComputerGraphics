#pragma once
#include <windows.h>
#include <map>
#include <string>
#include <osg/Matrixf>

namespace Common
{
	class FacadeFactoryA;
	class Facade;

	typedef std::map<std::string, FacadeFactoryA*> FacadeFactories;

	class FacadeManufactory
	{
	public:
		static void start();
		static void end();
		static FacadeManufactory* instance();


		bool addFactory(std::string sName, FacadeFactoryA* pFactory);

		Facade* create(std::string sFacadeFactory, std::string sName, osg::Node* pNode, osg::Matrixf mTrans = osg::Matrixf::identity(), bool bVisible = true);

	private:
		FacadeFactories m_mFactories;

		FacadeManufactory();
		virtual ~FacadeManufactory();
		static FacadeManufactory* sm_pManufactory;

	};
}
