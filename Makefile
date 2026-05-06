SHELL := /bin/zsh

POLICY_SOURCE_ROOT ?= isaac_policy_sources
POLICY_BUILD_VENV ?= .venv-policy-build
PRETRAINED_POLICY_RAW_BASE ?= https://raw.githubusercontent.com/tatsuya-ogawa/IsaacSim_pretrained_models/main
GO2_BACKFLIP_POLICY_RAW_URL ?= https://raw.githubusercontent.com/tatsuya-ogawa/IsaacSim-go2-backflip/main/assets/policy.pt
FETCH_POLICY_VARIANT ?= $(if $(POLICY_VARIANT),$(POLICY_VARIANT),all)

.PHONY: help anymal-usdz spot-usdz go2-usdz h1-usdz usdz fetch-policies policy-tooling compile-policy-model policy-model spot-policy-model anymal-policy-model anymal-rough-policy-model h1-policy-model go2-policy-model go2-rough-policy-model go2-backflip-policy-model

help:
	@echo "Available targets:"
	@echo "  make anymal-usdz  Build IsaacSwift/RobotAssets/anymal_c/anymal_c.usdz"
	@echo "  make spot-usdz    Build IsaacSwift/RobotAssets/spot/spot.usdz"
	@echo "  make go2-usdz     Build IsaacSwift/RobotAssets/go2/go2.usdz"
	@echo "  make h1-usdz      Build IsaacSwift/RobotAssets/h1/h1.usdz"
	@echo "  make usdz         Build anymal, spot, go2, and h1 USDZ assets"
	@echo "  make fetch-policies        Download policy sources into $(POLICY_SOURCE_ROOT)"
	@echo "  make policy-tooling        Create $(POLICY_BUILD_VENV) with torch and coremltools"
	@echo "  make compile-policy-model POLICY_VARIANT=spot|anymal|anymal_rough|h1|go2|go2_rough|go2_backflip  Build one policy bundle"
	@echo "  make policy-model          Fetch and build all policy bundles"
	@echo "  make spot-policy-model     Fetch and build PolicyModels/spot_policy.mlmodelc"
	@echo "  make anymal-policy-model   Fetch and build PolicyModels/anymal_policy.mlmodelc"
	@echo "  make anymal-rough-policy-model Fetch and build PolicyModels/anymal_rough_policy.mlmodelc"
	@echo "  make h1-policy-model       Fetch and build PolicyModels/h1_policy.mlmodelc"
	@echo "  make go2-policy-model      Fetch and build PolicyModels/go2_policy.mlmodelc"
	@echo "  make go2-rough-policy-model Fetch and build PolicyModels/go2_rough_policy.mlmodelc"
	@echo "  make go2-backflip-policy-model Fetch and build PolicyModels/go2_backflip_policy.mlmodelc"

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
	@POLICY_SOURCE_ROOT="$(POLICY_SOURCE_ROOT)" PRETRAINED_POLICY_RAW_BASE="$(PRETRAINED_POLICY_RAW_BASE)" GO2_BACKFLIP_POLICY_RAW_URL="$(GO2_BACKFLIP_POLICY_RAW_URL)" ./scripts/policies/fetch_isaac_policy_sources.sh "$(FETCH_POLICY_VARIANT)"

policy-tooling:
	@uv venv "$(POLICY_BUILD_VENV)"
	@uv pip install --python "$(POLICY_BUILD_VENV)/bin/python" torch coremltools pyyaml

compile-policy-model:
	@test -n "$(POLICY_VARIANT)" || (echo "POLICY_VARIANT is required" >&2; exit 1)
	@POLICY_SOURCE_ROOT="$(POLICY_SOURCE_ROOT)" POLICY_BUILD_VENV="$(POLICY_BUILD_VENV)" ./scripts/policies/build_policy_model.sh "$(POLICY_VARIANT)"

policy-model:
	@$(MAKE) fetch-policies
	@$(MAKE) POLICY_VARIANT=spot compile-policy-model
	@$(MAKE) POLICY_VARIANT=anymal compile-policy-model
	@$(MAKE) POLICY_VARIANT=anymal_rough compile-policy-model
	@$(MAKE) POLICY_VARIANT=h1 compile-policy-model
	@$(MAKE) POLICY_VARIANT=go2 compile-policy-model
	@$(MAKE) POLICY_VARIANT=go2_rough compile-policy-model
	@$(MAKE) POLICY_VARIANT=go2_backflip compile-policy-model

spot-policy-model:
	@$(MAKE) POLICY_VARIANT=spot fetch-policies
	@$(MAKE) POLICY_VARIANT=spot compile-policy-model

anymal-policy-model:
	@$(MAKE) POLICY_VARIANT=anymal fetch-policies
	@$(MAKE) POLICY_VARIANT=anymal compile-policy-model

anymal-rough-policy-model:
	@$(MAKE) POLICY_VARIANT=anymal_rough fetch-policies
	@$(MAKE) POLICY_VARIANT=anymal_rough compile-policy-model

h1-policy-model:
	@$(MAKE) POLICY_VARIANT=h1 fetch-policies
	@$(MAKE) POLICY_VARIANT=h1 compile-policy-model

go2-policy-model:
	@$(MAKE) POLICY_VARIANT=go2 fetch-policies
	@$(MAKE) POLICY_VARIANT=go2 compile-policy-model

go2-rough-policy-model:
	@$(MAKE) POLICY_VARIANT=go2_rough fetch-policies
	@$(MAKE) POLICY_VARIANT=go2_rough compile-policy-model

go2-backflip-policy-model:
	@$(MAKE) POLICY_VARIANT=go2_backflip fetch-policies
	@$(MAKE) POLICY_VARIANT=go2_backflip compile-policy-model
