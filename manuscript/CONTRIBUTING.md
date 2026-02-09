# Contributing to the SPIE Manuscript

This is the LaTeX source for our SPIE Defense + Security 2026 paper (DS112, Paper ID: 14030-26).

**Manuscript due: April 8, 2026**

---

## Section Ownership

Each section has a primary owner. **Only edit your assigned section(s).** If you need changes in another section, talk to the owner.

| Section | File Location | Owner |
|---------|--------------|-------|
| Abstract | `main.tex` (top) | Lead |
| 1. Introduction | `main.tex` | Lead |
| 2. Related Work | `main.tex` | Lead + Tracking (PINN lit review) |
| 3. System Architecture | `main.tex` | Middleware + QA |
| 4. RF-DETR Detection | `main.tex` | **Perception (DONE)** |
| 5. Depth Integration | `main.tex` | Tracking team |
| 6. PINN Prediction | `main.tex` | Tracking team |
| 7. Anticipatory Control | `main.tex` | Controls |
| 8. Experimental Setup | `main.tex` | Flight Ops |
| 9. Results | `main.tex` | Lead |
| 10. Discussion | `main.tex` | Lead |
| 11. Conclusion | `main.tex` | Lead |

Each section has a `% TODO:` comment at the top telling you what to write. Look for your role name.

---

## How to Edit

### What you need installed

- A LaTeX distribution. Options:
  - **Overleaf** (easiest — no install, works in browser): upload the `manuscript/` folder
  - **Ubuntu/Linux**: `sudo apt install texlive-full`
  - **Mac**: install [MacTeX](https://tug.org/mactex/)
  - **Windows**: install [MiKTeX](https://miktex.org/)
- A text editor (VS Code with LaTeX Workshop extension works great, or just any text editor)

### The file you edit

Everything is in **`main.tex`**. Open it, find your section (search for the section name), and write your content between the `\section{...}` and the next `\section{...}`.

### How to build the PDF

```bash
cd manuscript/
pdflatex main.tex
bibtex main
pdflatex main.tex
pdflatex main.tex
```

Yes, you run `pdflatex` three times. That's normal — LaTeX needs multiple passes for references and citations.

If you're using Overleaf, it builds automatically when you save.

---

## LaTeX Basics (Crash Course)

### Text formatting

```latex
\textbf{bold text}
\textit{italic text}
\texttt{monospace text}  % use for code, topic names, etc.
```

### Sections (already set up — don't add new ones without asking)

```latex
\section{Main Section}           % numbered: 1, 2, 3...
\subsection{Sub Section}         % numbered: 1.1, 1.2...
\subsubsection{Sub Sub Section}  % numbered: 1.1.1...
```

### Citing a reference

All references are in `references.bib`. To cite one:

```latex
We use the Kalman filter~\cite{kalmanfilter1960} for state estimation.
```

The `~` before `\cite` prevents a line break between the word and the citation number. **Always use `~\cite`.**

If you need to add a new reference, add it to `references.bib` using one of the existing entries as a template, or ask Lead.

### Figures

Figures are in the `figures/` folder. To include one:

```latex
\begin{figure}[htbp]
    \centering
    \includegraphics[width=\textwidth]{your_figure.pdf}
    \caption{Description of what the figure shows.}
    \label{fig:your_label}
\end{figure}
```

Then reference it in text with: `Figure~\ref{fig:your_label}`

**Use PDF figures when possible** (vector graphics, scales cleanly). PNG is OK for photos/screenshots.

### Tables

```latex
\begin{table}[htbp]
    \centering
    \caption{Description of the table.}
    \label{tab:your_label}
    \begin{tabular}{lcc}
        \hline
        \textbf{Column 1} & \textbf{Column 2} & \textbf{Column 3} \\
        \hline
        Row 1 data & 0.95 & 0.87 \\
        Row 2 data & 0.91 & 0.82 \\
        \hline
    \end{tabular}
\end{table}
```

- `l` = left-aligned column, `c` = centered, `r` = right-aligned
- `\\` ends a row
- `&` separates columns
- `\hline` draws a horizontal line

Reference in text with: `Table~\ref{tab:your_label}`

### Math

Inline math: `$x^2 + y^2 = z^2$`

Display math (centered, own line):
```latex
\begin{equation}
    \hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H \hat{x}_{k|k-1})
    \label{eq:kalman_update}
\end{equation}
```

### Lists

```latex
% Numbered list
\begin{enumerate}
    \item First thing
    \item Second thing
\end{enumerate}

% Bullet list
\begin{itemize}
    \item First thing
    \item Second thing
\end{itemize}
```

### Units (use siunitx for consistent formatting)

```latex
\SI{25}{FPS}
\SI{560x560}{pixels}    % doesn't work, just write $560 \times 560$
\SI{0.4}{\meter}
\SI{30}{\hertz}
```

Or just write the units in plain text — it's fine for a first draft.

---

## Common Mistakes to Avoid

1. **Don't leave blank lines inside a paragraph** — LaTeX treats a blank line as a new paragraph with indentation.
2. **Don't use `"quotes"` directly** — use `` ``quotes'' `` (two backticks to open, two single quotes to close).
3. **Percent signs need escaping** — write `50\%` not `50%` (bare `%` starts a comment).
4. **Ampersand needs escaping** — write `\&` not `&` (bare `&` is a column separator).
5. **Don't commit build artifacts** — `.aux`, `.log`, `.bbl`, `.blg`, `.out`, `.pdf` are gitignored.

---

## Git Workflow

1. Pull before you start: `git pull origin main`
2. Edit your section in `main.tex`
3. Build the PDF to make sure it compiles (see above)
4. Commit with a descriptive message:
   ```bash
   git add manuscript/main.tex
   git commit -m "docs(manuscript): draft Section 5 - Depth Integration"
   ```
5. Push: `git push origin main`

**If you get a merge conflict in `main.tex`:** don't panic. Since everyone edits different sections, conflicts should be rare. If it happens, the conflict markers (`<<<<<<<`, `=======`, `>>>>>>>`) show both versions — keep the right one and delete the markers.

---

## Adding Figures

1. Generate your figure as PDF (preferred) or PNG
2. Put it in `manuscript/figures/`
3. Reference it in `main.tex` (see the Figures section above)
4. Commit both the figure file and the `main.tex` change

If you have a Python script that generates figures, put it in `scripts/` (see `scripts/generate_spie_figures.py` for an example).

---

## Questions?

Ask Lead. Don't spend hours fighting LaTeX — ask for help.
