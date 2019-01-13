include_subdirs = $$files("*include", true)
include_absdirs =

for(include_dir, include_subdirs) {
    include_absdirs += $$absolute_path($$include_dir)
}

INCLUDEPATH += $$include_absdirs

ROS_ENV = $$(ROSPATH)
!isEmpty(ROS_ENV) INCLUDEPATH += $$ROS_ENV/include
