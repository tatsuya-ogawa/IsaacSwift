//
//  GameViewController.swift
//  IsaacSwift
//
//  Created by Tatsuya Ogawa on 2026/04/29.
//

import UIKit
import MetalKit
import SwiftUI

// Our iOS specific view controller
class GameViewController: UIViewController {

    var renderer: Renderer!
    var mtkView: MTKView!
    private var policyActionProvider: PolicyActionProvider?
    private var orbitPanGestureRecognizer: UIPanGestureRecognizer?
    private var pinchGestureRecognizer: UIPinchGestureRecognizer?
    private var debugHostingController: UIViewController?
    private var debugButton: UIButton?
    private var trackingButton: UIButton?
    private var currentSelection: RobotPolicySelection = .default

    override func viewDidLoad() {
        super.viewDidLoad()

        guard let mtkView = view as? MTKView else {
            print("View of Gameview controller is not an MTKView")
            showStatus(message: "MTKView unavailable", identifier: "renderer-status-label")
            return
        }

        // Select the device to render with.  We choose the default device
        guard let defaultDevice = MTLCreateSystemDefaultDevice() else {
            print("Metal is not supported")
            showStatus(message: "Metal is not supported", identifier: "renderer-status-label")
            return
        }
        
#if targetEnvironment(simulator)
        print("Metal 4 is not supported on simulator")
        showStatus(message: "Metal 4 is not supported on simulator", identifier: "renderer-status-label")
        installDebugButton()
        return
#else
        // Check for Metal 4 support
        if !defaultDevice.supportsFamily(.metal4) {
            print("Metal 4 is not supported")
            showStatus(message: "Metal 4 is not supported", identifier: "renderer-status-label")
            installDebugButton()
            return
        }
        
        mtkView.device = defaultDevice
        mtkView.backgroundColor = UIColor.black
        self.mtkView = mtkView

        guard buildRenderer(into: mtkView, selection: currentSelection) else {
            return
        }

        installOrbitGestureRecognizer(on: mtkView)
        installPinchGestureRecognizer(on: mtkView)
        installDebugButton()
        installTrackingButton()
#endif
    }

    /// Builds (or rebuilds) the Metal 4 renderer for the requested robot kind
    /// and wires it into `mtkView`. Returns false if the renderer fails to
    /// initialize so the caller can show a status message.
    @discardableResult
    private func buildRenderer(into mtkView: MTKView,
                               selection: RobotPolicySelection) -> Bool {
        let policyActionProvider = makePolicyActionProvider(selection: selection)

        guard let newRenderer = Renderer(metalKitView: mtkView,
                                         policyActionProvider: policyActionProvider,
                                         policyRuntimeConfiguration: selection.runtimeConfiguration,
                                         robotKind: selection.robotKind) else {
            print("Renderer cannot be initialized (kind=\(selection.robotKind.rawValue), policy=\(selection.policyKind.rawValue))")
            showStatus(message: "Renderer cannot be initialized",
                       identifier: "renderer-status-label")
            return false
        }

        self.policyActionProvider = policyActionProvider
        self.renderer = newRenderer
        self.currentSelection = RobotPolicySelection(robotKind: newRenderer.robotKind,
                                                     policyKind: selection.policyKind)
        mtkView.isAccessibilityElement = true
        mtkView.accessibilityIdentifier = "renderer-view"
        mtkView.accessibilityValue = "robot=\(newRenderer.robotKind.rawValue),policy=\(currentSelection.policyKind.rawValue)"

        renderer.mtkView(mtkView, drawableSizeWillChange: mtkView.drawableSize)
        mtkView.delegate = renderer
        return true
    }

    /// Tears down the existing debug overlay (if any), rebuilds the renderer
    /// with the requested robot kind, and reopens the overlay so the user
    /// stays in the debug flow.
    private func switchSelection(_ selection: RobotPolicySelection) {
        guard selection != currentSelection, let mtkView else { return }
        if let existing = debugHostingController {
            hideDebugOverlay(existing)
        }
        _ = buildRenderer(into: mtkView, selection: selection)
        showDebugOverlay()
    }

    private func showStatus(message: String, identifier: String) {
        view.backgroundColor = .black

        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.text = message
        label.textColor = .white
        label.textAlignment = .center
        label.numberOfLines = 0
        label.accessibilityIdentifier = identifier

        view.addSubview(label)
        NSLayoutConstraint.activate([
            label.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            label.centerYAnchor.constraint(equalTo: view.centerYAnchor),
            label.leadingAnchor.constraint(greaterThanOrEqualTo: view.leadingAnchor, constant: 24),
            label.trailingAnchor.constraint(lessThanOrEqualTo: view.trailingAnchor, constant: -24),
        ])
    }

    private func makePolicyActionProvider(selection: RobotPolicySelection) -> PolicyActionProvider? {
        do {
            let policyModelRunner = try PolicyModelRunner(configuration: selection.modelConfiguration)
            let actions = try policyModelRunner.predictActions(observations: policyModelRunner.zeroObservations())
            precondition(actions.count == selection.runtimeConfiguration.jointCount,
                         "Policy action count must match selected runtime joint count")
            print("Policy model loaded: \(policyModelRunner.configuration.resourceName) obs=\(policyModelRunner.observationCount) actions=\(actions.count)")
            return DemoPolicyActionProvider(runner: policyModelRunner,
                                            configuration: selection.runtimeConfiguration)
        } catch {
            fatalError("Policy model warm-up failed for \(selection.robotKind) / \(selection.policyKind): \(error)")
        }
    }

    private func installOrbitGestureRecognizer(on view: MTKView) {
        let recognizer = UIPanGestureRecognizer(target: self, action: #selector(handleOrbitPan(_:)))
        recognizer.maximumNumberOfTouches = 1
        view.addGestureRecognizer(recognizer)
        orbitPanGestureRecognizer = recognizer
    }

    @objc
    private func handleOrbitPan(_ recognizer: UIPanGestureRecognizer) {
        guard recognizer.state == .began || recognizer.state == .changed else {
            return
        }

        let delta = recognizer.translation(in: recognizer.view)
        renderer?.applyOrbitGesture(delta: delta)
        recognizer.setTranslation(.zero, in: recognizer.view)
    }

    private func installPinchGestureRecognizer(on view: MTKView) {
        let recognizer = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
        view.addGestureRecognizer(recognizer)
        pinchGestureRecognizer = recognizer
    }

    @objc
    private func handlePinch(_ recognizer: UIPinchGestureRecognizer) {
        guard recognizer.state == .began || recognizer.state == .changed else {
            return
        }
        
        renderer?.applyPinchGesture(scale: Float(recognizer.scale))
        recognizer.scale = 1.0
    }

    // MARK: - Physics Debug UI

    private func installDebugButton() {
        let button = UIButton(type: .system)
        button.setTitle("🔬 Debug", for: .normal)
        button.titleLabel?.font = .systemFont(ofSize: 14, weight: .medium)
        button.backgroundColor = UIColor.systemBackground.withAlphaComponent(0.85)
        button.layer.cornerRadius = 8
        var config = UIButton.Configuration.plain()
        config.contentInsets = NSDirectionalEdgeInsets(top: 6, leading: 12, bottom: 6, trailing: 12)
        button.configuration = config
        button.translatesAutoresizingMaskIntoConstraints = false
        button.addTarget(self, action: #selector(togglePhysicsDebug), for: .touchUpInside)
        view.addSubview(button)
        NSLayoutConstraint.activate([
            button.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 12),
            button.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -12),
        ])
        self.debugButton = button
    }

    private func installTrackingButton() {
        let button = UIButton(type: .system)
        button.setTitle("🎥 Track Off", for: .normal)
        button.titleLabel?.font = .systemFont(ofSize: 14, weight: .medium)
        button.backgroundColor = UIColor.systemBackground.withAlphaComponent(0.85)
        button.layer.cornerRadius = 8
        var config = UIButton.Configuration.plain()
        config.contentInsets = NSDirectionalEdgeInsets(top: 6, leading: 12, bottom: 6, trailing: 12)
        button.configuration = config
        button.translatesAutoresizingMaskIntoConstraints = false
        button.addTarget(self, action: #selector(toggleTracking), for: .touchUpInside)
        view.addSubview(button)
        NSLayoutConstraint.activate([
            button.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 12),
            button.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 12),
        ])
        self.trackingButton = button
    }

    @objc
    private func toggleTracking() {
        guard let renderer = renderer else { return }
        renderer.isTrackingEnabled.toggle()
        
        let title = renderer.isTrackingEnabled ? "🎥 Track On" : "🎥 Track Off"
        trackingButton?.setTitle(title, for: .normal)
        trackingButton?.tintColor = renderer.isTrackingEnabled ? .systemBlue : nil
    }

    @objc
    private func togglePhysicsDebug() {
        if let existing = debugHostingController {
            hideDebugOverlay(existing)
        } else {
            showDebugOverlay()
        }
    }

    private func showDebugOverlay() {
        guard let sharedLoop = renderer?.policyPhysicsLoop else { return }
        
        let debugView = PhysicsDebugView(sharedLoop: sharedLoop,
                                         selection: currentSelection,
                                         onClose: { [weak self] in
            if let existing = self?.debugHostingController {
                self?.hideDebugOverlay(existing)
            }
        }, onRequestSelectionSwitch: { [weak self] selection in
            self?.switchSelection(selection)
        })
        
        let hostingController = UIHostingController(rootView: debugView)
        hostingController.view.backgroundColor = .clear
        
        addChild(hostingController)
        view.addSubview(hostingController.view)
        hostingController.view.translatesAutoresizingMaskIntoConstraints = false
        
        NSLayoutConstraint.activate([
            hostingController.view.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            hostingController.view.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            hostingController.view.bottomAnchor.constraint(equalTo: view.bottomAnchor),
        ])
        
        hostingController.didMove(toParent: self)
        debugHostingController = hostingController
        
        debugButton?.setTitle("Close Debug", for: .normal)
        debugButton?.tintColor = .systemRed
    }

    private func hideDebugOverlay(_ controller: UIViewController) {
        controller.willMove(toParent: nil)
        controller.view.removeFromSuperview()
        controller.removeFromParent()
        debugHostingController = nil
        
        debugButton?.setTitle("🔬 Debug", for: .normal)
        debugButton?.tintColor = nil
    }
}
