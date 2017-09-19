

SOURCES = $(wildcard *.tex)
SOURCES += $(wildcard *.bib)
SOURCES += diagram.pdf

SystemTemplate.pdf: $(SOURCES)
	echo "Sources are: $(SOURCES)"
	pdflatex SystemTemplate
	bibtex SystemTemplate
	pdflatex SystemTemplate
	pdflatex SystemTemplate


diagram.pdf: diagram.eps
	epstopdf diagram.eps

diagram.eps: diagram.fig
	fig2dev -L eps diagram.fig diagram.eps


clean:
	rm -f *.aux *.bbl *.blg *.loa *.lof *.log *.lot *.out SystemTemplate.pdf *.toc *~
