SRCS := $(wildcard *.tex)
PDFS := $(SRCS:.tex=.pdf)
PNGS := $(SRCS:.tex=.png)
SVGS := $(SRCS:.tex=.svg)

PNG_DPI ?= 160

.PHONY: all svg clean rx tx
all: $(PDFS) $(PNGS)
svg: $(SVGS)

rx: rx_frontend.pdf rx_frontend.png
tx: tx_frontend.pdf tx_frontend.png

%.pdf: %.tex
	pdflatex -interaction=nonstopmode -halt-on-error $< >/dev/null

%.png: %.pdf
	pdftoppm -png -r $(PNG_DPI) -singlefile $< $*

%.svg: %.pdf
	inkscape $< --export-type=svg --export-plain-svg --export-filename=$@ >/dev/null 2>&1

clean:
	rm -f *.aux *.log *.out *.fls *.fdb_latexmk *.pdf *.png *.svg
