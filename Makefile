SHELL := /bin/zsh

POLICY_SOURCE_ROOT ?= isaac_policy_sources
POLICY_BUILD_VENV ?= .venv-policy-build

.PHONY: help anymal-usdz spot-usdz go2-usdz h1-usdz usdz fetch-policies policy-tooling compile-policy-model policy-model spot-policy-model anymal-policy-model h1-policy-model

help:
	@echo "Available targets:"
	@echo "  make anymal-usdz  Build IsaacSwift/RobotAssets/anymal_c/anymal_c.usdz"
	@echo "  make spot-usdz    Build IsaacSwift/RobotAssets/spot/spot.usdz"
	@echo "  make go2-usdz     Build IsaacSwift/RobotAssets/go2/go2.usdz"
	@echo "  make h1-usdz      Build IsaacSwift/RobotAssets/h1/h1.usdz"
	@echo "  make usdz         Build anymal, spot, go2, and h1 USDZ assets"
	@echo "  make fetch-policies        Download Spot, ANYmal, and H1 policy sources from NVIDIA"
	@echo "  make policy-tooling        Create $(POLICY_BUILD_VENV) with torch and coremltools"
	@echo "  make compile-policy-model POLICY_VARIANT=spot|anymal|h1  Build one policy bundle"
	@echo "  make policy-model          Build PolicyModels/spot_policy.mlmodelc, anymal_policy.mlmodelc, and h1_policy.mlmodelc"
	@echo "  make spot-policy-model     Fetch and build PolicyModels/spot_policy.mlmodelc"
	@echo "  make anymal-policy-model   Fetch and build PolicyModels/anymal_policy.mlmodelc"
	@echo "  make h1-policy-model       Fetch and build PolicyModels/h1_policy.mlmodelc"

anymal-usdz:
	@./scripts/usdz/package_anymal_usdz.sh

spot-usdz:
	@./scripts/usdz/package_spot_usdz.sh

go2-usdz:
	@./scripts/usdz/package_go2_usdz.sh

h1-usdz:
	@./scripts/usdz/package_h1_usdz.sh

usdz: anymal-usdz spot-usdz go2-usdz h1-usdz

fetch-policies:
	@POLICY_SOURCE_ROOT="$(POLICY_SOURCE_ROOT)" ./scripts/policies/fetch_isaac_policy_sources.sh all

policy-tooling:
	@uv venv "$(POLICY_BUILD_VENV)"
	@uv pip install --python "$(POLICY_BUILD_VENV)/bin/python" torch coremltools pyyaml

compile-policy-model:
	@test -n "$(POLICY_VARIANT)" || (echo "POLICY_VARIANT is required" >&2; exit 1)
	@POLICY_SOURCE_ROOT="$(POLICY_SOURCE_ROOT)" POLICY_BUILD_VENV="$(POLICY_BUILD_VENV)" ./scripts/policies/build_policy_model.sh "$(POLICY_VARIANT)"

policy-model:
	@$(MAKE) POLICY_VARIANT=spot compile-policy-model
	@$(MAKE) POLICY_VARIANT=anymal compile-policy-model
	@$(MAKE) POLICY_VARIANT=h1 compile-policy-model

spot-policy-model:
	@$(MAKE) POLICY_VARIANT=spot fetch-policies
	@$(MAKE) POLICY_VARIANT=spot compile-policy-model

anymal-policy-model:
	@$(MAKE) POLICY_VARIANT=anymal fetch-policies
	@$(MAKE) POLICY_VARIANT=anymal compile-policy-model

h1-policy-model:
	@$(MAKE) POLICY_VARIANT=h1 fetch-policies
	@$(MAKE) POLICY_VARIANT=h1 compile-policy-model
