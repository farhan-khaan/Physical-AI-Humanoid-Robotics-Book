# SpecifyPlus - AI-Assisted Development Workflow

SpecifyPlus is a structured workflow system for managing feature development with AI assistance.

## Quick Start

### Initialize a New Feature

```bash
npm run sp:new
```

This creates a new feature branch and directory structure in `specs/`.

### Available Commands

- **`npm run sp:new`** - Create a new feature branch and spec directory
- **`npm run sp:plan`** - Set up or update the implementation plan
- **`npm run sp:check`** - Check prerequisites and validate feature setup
- **`npm run sp:context`** - Update agent context files

## Directory Structure

```
.specify/
├── memory/           # Project-wide memory and constitution
├── scripts/          # Automation scripts
│   └── powershell/  # PowerShell scripts for Windows/cross-platform
└── templates/       # Templates for specs, plans, ADRs, etc.

specs/
└── [NNN-feature-name]/  # Individual feature directories
    ├── spec.md          # Feature specification
    ├── plan.md          # Implementation plan
    ├── tasks.md         # Task breakdown
    └── ...              # Additional feature docs
```

## Workflow

1. **Create Feature**: `npm run sp:new` → creates feature branch and directory
2. **Write Spec**: Document requirements in `spec.md`
3. **Plan Implementation**: Use `npm run sp:plan` to structure the work
4. **Track Progress**: Update `tasks.md` as you work
5. **Document Decisions**: Add ADRs for architectural decisions

## Templates

All templates are in `.specify/templates/`:
- `spec-template.md` - Feature specification
- `plan-template.md` - Implementation plan
- `tasks-template.md` - Task tracking
- `adr-template.md` - Architecture Decision Record
- `checklist-template.md` - Quality checklist
- `phr-template.prompt.md` - Pull request helper

## Customization

Edit `.specify/memory/constitution.md` to define your project's core principles and development standards.
