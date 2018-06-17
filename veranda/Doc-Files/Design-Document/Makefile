SOURCES = $(wildcard *.tex)

DesignDocument.pdf: $(SOURCES)
	echo "Sources are: $(SOURCES)"
	pdflatex DesignDocument
	pdflatex DesignDocument

clean:
	rm -f *.aux *.bbl *.blg *.loa *.lof *.log *.lot *.out DesignDocument.pdf *.toc *~ *.fls
