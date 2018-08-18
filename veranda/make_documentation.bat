@ECHO OFF

:: Windows bat script to build the documentation for this project
:: Requirements (These must be in your PATH):
:: * doxygen
:: * pdflatex
:: * sphinx

set PROJECTNAME=Veranda

:: Make output folder for results
mkdir Documentation

if "%1" == "doxygen" goto doxygen
if "%1" == "contract" goto contract
if "%1" == "sphinx" goto sphinx
if "%1" == "design" goto design

:: Generate ./Documentation/Doxygen/html and ./Documentation/Doxygen/html/latex
:doxygen
doxygen

:: Generate reference doc PDF from doxygen output
cd Doc-Files/Latex-Reference
pdflatex refman
pdflatex refman
copy refman.pdf Doxygen-PDF-Reference.pdf
robocopy ./ ../../Documentation Doxygen-PDF-Reference.pdf /copy:dat
cd ../..

if "%1" neq "" goto end

:: Generate PDF from Contract
:contract
cd Doc-Files/Contract
pdflatex contract
pdflatex contract
cd ../..

if "%1" neq "" goto end

:: Generate Sphinx Website
:sphinx
cd Doc-Files/User-Manual
sphinx-build -M html source build
robocopy build/html ../../Documentation/Sphinx-Web-User-Manual /e /purge /move
cd ../..

if "%2" == "--no-latex" goto end

:: Generate Sphinx Latex
cd Doc-Files/User-Manual
sphinx-build -M latex source build
cd build/latex
pdflatex %PROJECTNAME%
pdflatex %PROJECTNAME%
copy %PROJECTNAME%.pdf Sphinx-PDF-User-Manual.pdf
robocopy ./ ../../../../Documentation Sphinx-PDF-User-Manual.pdf /copy:dat
robocopy ./ ../../ Sphinx-PDF-User-Manual.pdf /copy:dat
cd ../../../..

if "%1" neq "" goto end

:: Generate PDF from Design Document
:design
cd Doc-Files/Design-Document
pdflatex DesignDocument
pdflatex DesignDocument
copy DesignDocument.pdf Design-Document.pdf
robocopy ./ ../../Documentation/ Design-Document.pdf /copy:dat
cd ../..

:end