// MathJax v3 config for pymdownx.arithmatex (generic mode).
// arithmatex renders source $...$ / $$...$$ into \(...\) / \[...\] delimiters.
// `\boldsymbol` (used for every Greek vector here) and `\checkmark` are NOT in
// the default tex-mml-chtml package set — without these two lines they render
// as red "undefined control sequence" text.
window.MathJax = {
  loader: { load: ["[tex]/boldsymbol"] },
  tex: {
    packages: { "[+]": ["boldsymbol"] },
    inlineMath: [["\\(", "\\)"]],
    displayMath: [["\\[", "\\]"]],
    processEscapes: true,
    processEnvironments: true,
  },
  options: {
    ignoreHtmlClass: ".*|",
    processHtmlClass: "arithmatex",
  },
};

// Re-typeset on instant-navigation page swaps (Material's document$ stream).
document$.subscribe(() => {
  MathJax.startup.output.clearCache();
  MathJax.typesetClear();
  MathJax.texReset();
  MathJax.typesetPromise();
});
