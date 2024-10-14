#pragma once
#include <windows.h>
#include <osg/Object>

#include <Common/Facade.h>

class SelectionHandler: public osg::Object
{
public:
	SelectionHandler();
	SelectionHandler(Common::Facade* pFacade);
	virtual ~SelectionHandler();

	// Inherited via Object
	virtual Object* cloneType() const override;
	virtual Object* clone(const osg::CopyOp&) const override;
	virtual const char* libraryName() const override;
	virtual const char* className() const override;

	Common::Facade* facade();

protected:
	Common::Facade* m_pFacade;

};

