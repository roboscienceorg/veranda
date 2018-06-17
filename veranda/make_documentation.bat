:: Windows bat script to build the documentation for this project
:: Requirements (These must be in your PATH):
:: * doxygen
:: * pdflatex
:: * sphinx

set PROJECTNAME=Veranda

:: Make output folder for results
mkdir Documentation

:: Generate ./Documentation/Doxygen/html and ./Documentation/Doxygen/html/latex
doxygen

:: Generate reference doc PDF from doxygen output
cd Doc-Files/Latex-Reference
pdflatex refman
pdflatex refman
copy /Y refman.pdf Doxygen-PDF-Reference.pdf
move /Y Doxygen-PDF-Reference.pdf ../../Documentation/Doxygen-PDF-Reference.pdf
cd ../..

:: Generate PDF from Contract
cd Doc-Files/Contract
pdflatex contract
pdflatex contract
cd ../..

:: Generate Sphinx Website
cd Doc-Files/User-Manual
sphinx-build -M html source build
move /Y build/html ../../Documentation/Sphinx-Web-User-Manual

:: Generate Sphinx Latex
sphinx-build -M latex source build
cd build/latex
pdflatex %PROJECTNAME%
pdflatex %PROJECTNAME%
copy /Y %PROJECTNAME%.pdf Sphinx-PDF-User-Manual.pdf
move Sphinx-PDF-User-Manual.pdf ../../../../Documentation/Sphinx-PDF-User-Manual.pdf
copy /Y %PROJECTNAME%.pdf Sphinx-PDF-User-Manual.pdf
move Sphinx-PDF-User-Manual.pdf ../../Sphinx-PDF-User-Manual.pdf
cd ../../../..

:: Generate PDF from Design Document
cd Doc-Files/Design-Document
pdflatex DesignDocument
pdflatex DesignDocument
copy /Y DesignDocument.pdf Design-Document.pdf
move /Y Design-Document.pdf ../../Documentation/Design-Document.pdf
cd ../..