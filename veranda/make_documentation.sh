#!/bin/bash

# Unix bash script to build the documentation for this project
# Requirements (These must be in your PATH):
# * doxygen
# * pdflatex
# * sphinx

PROJECTNAME="Veranda"

# Make output folder for results
mkdir -p Documentation

# Generate ./Documentation/Doxygen/html and ./Documentation/Doxygen/html/latex
function build_doxygen
{
    doxygen

    # Generate reference doc PDF from doxygen output
    cd Doc-Files/Latex-Reference
    pdflatex refman
    pdflatex refman
    cp refman.pdf ../../Documentation/Doxygen-PDF-Reference.pdf
    cd ../..
}

# Generate PDF from Contract
function build_contract
{
    cd Doc-Files/Contract
    pdflatex contract
    pdflatex contract
    cd ../..
}

# Generate sphinx webpage and pdf
function build_sphinx
{
    # Generate Sphinx Website
    cd Doc-Files/User-Manual
    sphinx-build -M html source build
    rsync -a build/html/ ../../Documentation/Sphinx-Web-User-Manual/
    rm -rf build/html

    # Generate Sphinx Latex
    sphinx-build -M latex source build
    cd build/latex
    pdflatex $PROJECTNAME
    pdflatex $PROJECTNAME
    cp $PROJECTNAME.pdf ../../../../Documentation/Sphinx-PDF-User-Manual.pdf
    cp $PROJECTNAME.pdf ../../Sphinx-PDF-User-Manual.pdf
    cd ../../../..
}

# Generate PDF from Design Document
function build_design
{
    cd Doc-Files/Design-Document
    pdflatex DesignDocument
    pdflatex DesignDocument
    cp DesignDocument.pdf ../../Documentation/Design-Document.pdf
    cd ../..
}

if [ -z "$1" ] || [ "$1" = "doxygen" ]; then
    build_doxygen
fi

if [ -z "$1" ] || [ "$1" = "sphinx" ]; then
    build_sphinx
fi

if [ -z "$1" ] || [ "$1" = "contract" ]; then
    build_contract
fi

if [ -z "$1" ] || [ "$1" = "design" ]; then
    build_design
fi
