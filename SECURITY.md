# Security Policy

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 1.x     | :white_check_mark: |
| < 1.0   | :x:                |

## Reporting a Vulnerability

If you discover a security vulnerability within this library, please follow responsible disclosure:

1. **Do NOT** open a public GitHub issue.
2. Email the maintainer at: `info@thymos.cz`.
3. Include:
   - A description of the vulnerability
   - Steps to reproduce
   - Potential impact
   - Any suggested fixes (optional)

We will acknowledge receipt within 48 hours and aim to provide a fix or mitigation within 14 days for critical issues.

## Scope

This library is designed for embedded systems. Security considerations include:
- No dynamic memory allocation in steady state (reduces attack surface)
- No network code (networking is out of scope for this library)
- No MCU-side NVS/filesystem ownership
- Explicit EE871 custom-memory writes can persist sensor configuration

## Security Best Practices for Users

- Always validate external inputs before passing to `Config`
- Restrict persistent-write APIs and diagnostic CLI commands to authorized
  maintenance workflows
- Use hardware watchdogs in production deployments
- Keep dependencies updated
