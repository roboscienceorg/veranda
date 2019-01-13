TEMPLATE = subdirs

pkgs = $$files(*.pro, true)
pkgs = $$replace(pkgs, all_packages.pro, )

for(pro_file, pkgs) {
    file_base = $$basename(pro_file)
    pkg_name = $$replace(file_base, .pro, )

    SUBDIRS += $$pkg_name
    $${pkg_name}.file = $$pro_file
}

DISTFILES += include_paths.pri
