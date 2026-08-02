Read and apply CODING_GUIDELINES.md.

Always use `bazelisk`, not `bazel`.

Do not add superficial comments or docstrings that restate what is clear from naming of variables/functions/arguments, unless explicitly asked to. Do not comment on what is already clear from the implementation. Only comment on non-trivial aspects that cannot be deduced by naming.

Do not add comments to Bazel targets stating what they do.

Do not add references to intermediate milestones or to any untracked documents into source code or tracked documentation.

Do not keep dead code or test-only code in production code files. Move test-only code either directly into test suites or, if reused in multiple test suites, into suitable `testonly=True` Bazel targets.
