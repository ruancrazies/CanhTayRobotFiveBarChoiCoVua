"""Repository configuration. A regular package (not a namespace package) on purpose.

``config/protocol_commands.py`` is imported by the PC code and read by the protocol cross-check
test, so it must not be shadowed by an unrelated installed package that is also called ``config``.
Run the application and the tests from the repository root.
"""
