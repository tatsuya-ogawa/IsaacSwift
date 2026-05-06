//
//  PhysicsDebugView.swift
//  IsaacSwift
//
//  Step-by-step physics debug UI. Lets you advance the policy/physics loop
//  one tick at a time and inspect all relevant state. Copy-to-clipboard
//  exports a text snapshot suitable for pasting into a chat for analysis.
//

import SwiftUI
import Combine
import simd

@MainActor
final class PhysicsDebugViewModel: ObservableObject {

    @Published var loop: PolicyPhysicsLoop
    private let ownsLoop: Bool
    @Published var stepCount: Int = 0
    @Published var simTime: Double = 0
    @Published var obs: ObservationSnapshot = .zero
    @Published var rawActions: [Float] = Array(repeating: 0, count: 12)
    @Published var scaledActions: [Float] = Array(repeating: 0, count: 12)
    @Published var jointDeltas: [Float] = Array(repeating: 0, count: 12)
    @Published var hasJointActuator: Bool = false
    @Published var isRunning: Bool = false
    @Published var command: SIMD3<Float>
    /// Read-only mirror of the simulator's robot kind. In standalone mode the
    /// user can switch this via `switchRobotKind(_:)`. In shared mode it
    /// mirrors the renderer's robot/policy choice.
    @Published private(set) var selection: RobotPolicySelection

    /// Optional external switcher used in shared mode. The host (e.g.
    /// GameViewController) supplies this to recreate the renderer with a new
    /// robot/policy pair, since the shared loop is owned by the renderer.
    var externalSelectionSwitcher: ((RobotPolicySelection) -> Void)?

    /// True iff this view model can rebuild its loop directly. In shared
    /// mode an external switcher may still be available; check
    /// `canRequestRobotKindSwitch` for the broader UI test.
    var canSwitchRobotKind: Bool { ownsLoop }

    /// True iff the picker should be shown — either we own the loop or the
    /// host installed an external switcher.
    var canRequestRobotKindSwitch: Bool {
        ownsLoop || externalSelectionSwitcher != nil
    }

    private var runTask: Task<Void, Never>?

    var robotKind: IsaacSwiftRobotKind {
        selection.robotKind
    }

    var policyKind: RobotPolicyKind {
        selection.policyKind
    }

    var availablePolicies: [RobotPolicyKind] {
        robotKind.modelDefinition.policyOptions
    }

    struct ObservationSnapshot {
        var basePos: SIMD3<Float> = .zero
        var baseQuat: SIMD4<Float> = SIMD4<Float>(0, 0, 0, 1)
        var baseLinVelB: SIMD3<Float> = .zero
        var baseAngVelB: SIMD3<Float> = .zero
        var gravityB: SIMD3<Float> = SIMD3<Float>(0, 0, -1)
        var jointPositionDeltas: [Float] = []
        var jointVelocities: [Float] = []
        var uprightZ: Float = 1.0

        static let zero = ObservationSnapshot()
    }

    var jointNames: [String] {
        robotKind.modelDefinition.articulationProfile.policyJointBindings
            .sorted { $0.actionIndex < $1.actionIndex }
            .map { binding in
                String(binding.nodePath.split(separator: "/").last ?? "?")
            }
    }

    /// Use the shared loop from the renderer (pauses it for stepping).
    init(sharedLoop: PolicyPhysicsLoop,
         selection: RobotPolicySelection) {
        self.loop = sharedLoop
        self.ownsLoop = false
        self.selection = RobotPolicySelection(robotKind: sharedLoop.simulator.robotKind,
                                             policyKind: selection.policyKind)
        self.command = sharedLoop.configuration.defaultCommand
        self.stepCount = sharedLoop.stepCount
        self.simTime = Double(sharedLoop.stepCount) * sharedLoop.configuration.policyUpdateInterval
        loop.paused = true
        refreshSnapshot()
    }

    /// Standalone mode — creates its own loop (used in previews or when no renderer).
    /// Defaults to Spot to match the bundled policy.
    init(selection explicitSelection: RobotPolicySelection? = nil) {
        let selection = explicitSelection ?? RobotPolicySelection(robotKind: RobotModelDefinitions.defaultKind)
        let runtime = selection.runtimeConfiguration
        let runner = try? PolicyModelRunner(configuration: selection.modelConfiguration)
        let provider: PolicyActionProvider?
        if let runner {
            provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: runtime,
                                                command: runtime.defaultCommand)
        } else {
            provider = nil
        }
        self.loop = PolicyPhysicsLoop(robotKind: selection.robotKind,
                                      configuration: runtime,
                                      provider: provider)
        self.ownsLoop = true
        self.selection = selection
        self.command = runtime.defaultCommand
        loop.paused = false
        loop.reset()
        refreshSnapshot()
    }

    /// Rebuilds the internal loop with a new robot kind. Only valid in
    /// standalone mode (where this view model owns its loop).
    func switchRobotKind(_ kind: IsaacSwiftRobotKind) {
        switchSelection(RobotPolicySelection(robotKind: kind,
                                             policyKind: selection.policyKind))
    }

    func switchPolicyKind(_ policyKind: RobotPolicyKind) {
        switchSelection(RobotPolicySelection(robotKind: robotKind,
                                             policyKind: policyKind))
    }

    private func switchSelection(_ nextSelection: RobotPolicySelection) {
        guard nextSelection != selection else { return }

        // Shared mode: delegate to the host so it can rebuild the renderer.
        if !ownsLoop {
            externalSelectionSwitcher?(nextSelection)
            return
        }
        stopContinuous()

        let runtime = nextSelection.runtimeConfiguration
        let runner = try? PolicyModelRunner(configuration: nextSelection.modelConfiguration)
        let provider: PolicyActionProvider?
        if let runner {
            provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: runtime,
                                                command: runtime.defaultCommand)
        } else {
            provider = nil
        }
        self.loop = PolicyPhysicsLoop(robotKind: nextSelection.robotKind,
                                      configuration: runtime,
                                      provider: provider)
        self.selection = nextSelection
        self.command = runtime.defaultCommand
        self.stepCount = 0
        self.simTime = 0
        loop.paused = true
        refreshSnapshot()
    }

    func stepOnce() {
        _ = loop.stepOnePolicyTick()
        stepCount = loop.stepCount
        simTime = Double(stepCount) * loop.configuration.policyUpdateInterval
        loop.paused = true
        refreshSnapshot()
    }

    func stepN(_ n: Int) {
        for _ in 0..<n {
            stepOnce()
        }
    }

    func reset() {
        stopContinuous()
        loop.reset()
        stepCount = 0
        simTime = 0
        loop.paused = true
        refreshSnapshot()
    }

    func startContinuous() {
        guard !isRunning else { return }
        isRunning = true
        loop.paused = true
        runTask = Task { @MainActor in
            while isRunning && !Task.isCancelled {
                let dt = loop.configuration.policyUpdateInterval
                _ = loop.stepOnePolicyTick()
                stepCount = loop.stepCount
                simTime = Double(stepCount) * dt
                refreshSnapshot()
                try? await Task.sleep(nanoseconds: UInt64(dt * 1_000_000_000))
            }
        }
    }

    func stopContinuous() {
        isRunning = false
        runTask?.cancel()
        runTask = nil
        loop.paused = true
    }

    /// Called when the debug view is dismissed — resumes normal rendering.
    func resumeLoop() {
        stopContinuous()
        loop.paused = false
    }

    func copyToClipboard() {
        let text = formattedSnapshot()
        #if canImport(UIKit)
        UIPasteboard.general.string = text
        #elseif canImport(AppKit)
        NSPasteboard.general.clearContents()
        NSPasteboard.general.setString(text, forType: .string)
        #endif
    }

    func formattedSnapshot() -> String {
        var lines: [String] = []
        lines.append("=== Physics Debug Snapshot ===")
        lines.append("step: \(stepCount)  time: \(String(format: "%.4f", simTime))s")
        lines.append("")
        lines.append("-- Base State --")
        lines.append("pos_world:    (\(f3(obs.basePos)))")
        lines.append("quat_xyzw:   (\(f4(obs.baseQuat)))")
        lines.append("lin_vel_b:   (\(f3(obs.baseLinVelB)))")
        lines.append("ang_vel_b:   (\(f3(obs.baseAngVelB)))")
        lines.append("gravity_b:   (\(f3(obs.gravityB)))")
        lines.append("upright_z:   \(String(format: "%.4f", obs.uprightZ))")
        lines.append("")
        lines.append("-- Joints (delta / vel / raw_act / scaled_act) --")
        let names = jointNames
        let count = min(names.count,
                        obs.jointPositionDeltas.count,
                        obs.jointVelocities.count,
                        rawActions.count,
                        scaledActions.count)
        for i in 0..<count {
            let name = names[i].padding(toLength: 24, withPad: " ", startingAt: 0)
            let d = String(format: "%+.4f", obs.jointPositionDeltas[i])
            let v = String(format: "%+.3f", obs.jointVelocities[i])
            let r = String(format: "%+.4f", rawActions[i])
            let s = String(format: "%+.4f", scaledActions[i])
            lines.append("\(name)  Δ=\(d)  ω=\(v)  raw=\(r)  scl=\(s)")
        }
        lines.append("")
        lines.append("-- Summary --")
        let maxDelta = obs.jointPositionDeltas.map { abs($0) }.max() ?? 0
        let maxVel = obs.jointVelocities.map { abs($0) }.max() ?? 0
        lines.append("max|Δ|: \(String(format: "%.4f", maxDelta))  max|ω|: \(String(format: "%.2f", maxVel))")
        lines.append("base_z: \(String(format: "%.4f", obs.basePos.z))")
        lines.append("command: (\(f3(command)))")
        lines.append("actuator: \(hasJointActuator ? "installed" : "missing")")
        lines.append("==============================")
        return lines.joined(separator: "\n")
    }

    private func refreshSnapshot() {
        let o = loop.simulator.currentObservation()
        let q = o.baseOrientationWorldXYZW
        obs = ObservationSnapshot(
            basePos: o.basePositionWorld,
            baseQuat: q,
            baseLinVelB: o.baseLinearVelocityBody,
            baseAngVelB: o.baseAngularVelocityBody,
            gravityB: o.gravityDirectionBody,
            jointPositionDeltas: o.jointPositionDeltas.map { $0.floatValue },
            jointVelocities: o.jointVelocities.map { $0.floatValue },
            uprightZ: 1 - 2 * (q.x * q.x + q.y * q.y)
        )
        rawActions = loop.lastRawActions
        scaledActions = rawActions.map { $0 * loop.actionScale }
        jointDeltas = loop.lastJointDeltas
        hasJointActuator = loop.hasJointActuator
    }

    private func f3(_ v: SIMD3<Float>) -> String {
        String(format: "%.4f, %.4f, %.4f", v.x, v.y, v.z)
    }

    private func f4(_ v: SIMD4<Float>) -> String {
        String(format: "%.4f, %.4f, %.4f, %.4f", v.x, v.y, v.z, v.w)
    }
}

struct PhysicsInspectorView: View {
    @ObservedObject var vm: PhysicsDebugViewModel
    @State private var copied = false
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationView {
            ScrollView {
                VStack(alignment: .leading, spacing: 16) {
                    baseStateSection
                    jointTable
                    summarySection
                }
                .padding()
            }
            .navigationTitle("Physics Inspector")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button(copied ? "Copied!" : "📋 Copy") {
                        vm.copyToClipboard()
                        copied = true
                        DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) {
                            copied = false
                        }
                    }
                    .foregroundColor(copied ? .green : .blue)
                }
                ToolbarItem(placement: .navigationBarLeading) {
                    Button("Close") { dismiss() }
                }
            }
            .font(.system(.caption, design: .monospaced))
        }
    }

    private var baseStateSection: some View {
        VStack(alignment: .leading, spacing: 6) {
            sectionHeader("Base State")
            row("pos_world", f3(vm.obs.basePos))
            row("lin_vel_b", f3(vm.obs.baseLinVelB))
            row("ang_vel_b", f3(vm.obs.baseAngVelB))
            row("gravity_b", f3(vm.obs.gravityB))
            HStack {
                row("upright_z", String(format: "%.4f", vm.obs.uprightZ))
                uprightIndicator(vm.obs.uprightZ)
            }
        }
        .padding(12)
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }

    private var jointTable: some View {
        VStack(alignment: .leading, spacing: 4) {
            sectionHeader("Joints")
            HStack(spacing: 0) {
                Text("Joint").frame(width: 60, alignment: .leading)
                Text("Δ[rad]").frame(width: 70, alignment: .trailing)
                Text("ω[r/s]").frame(width: 70, alignment: .trailing)
                Text("raw").frame(width: 60, alignment: .trailing)
                Text("scaled").frame(width: 60, alignment: .trailing)
            }
            .foregroundColor(.secondary)
            .padding(.bottom, 4)

            let names = vm.jointNames
            let count = min(names.count,
                            vm.obs.jointPositionDeltas.count,
                            vm.obs.jointVelocities.count,
                            vm.rawActions.count,
                            vm.scaledActions.count)
            ForEach(0..<count, id: \.self) { i in
                HStack(spacing: 0) {
                    Text(names[i])
                        .frame(width: 60, alignment: .leading)
                    coloredValue(vm.obs.jointPositionDeltas[i], threshold: 1.5)
                        .frame(width: 70, alignment: .trailing)
                    coloredValue(vm.obs.jointVelocities[i], threshold: 30)
                        .frame(width: 70, alignment: .trailing)
                    Text(String(format: "%+.3f", vm.rawActions[i]))
                        .frame(width: 60, alignment: .trailing)
                    Text(String(format: "%+.3f", vm.scaledActions[i]))
                        .frame(width: 60, alignment: .trailing)
                }
            }
        }
        .padding(12)
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }

    private var summarySection: some View {
        VStack(alignment: .leading, spacing: 6) {
            sectionHeader("Summary")
            let maxD = vm.obs.jointPositionDeltas.map { abs($0) }.max() ?? 0
            let maxV = vm.obs.jointVelocities.map { abs($0) }.max() ?? 0
            row("max|Δ|", String(format: "%.4f rad", maxD))
            row("max|ω|", String(format: "%.2f rad/s", maxV))
            row("base_z", String(format: "%.4f m", vm.obs.basePos.z))
            row("command", String(format: "(%.1f, %.1f, %.1f)", vm.command.x, vm.command.y, vm.command.z))
            row("actuator", vm.hasJointActuator ? "installed" : "missing")
        }
        .padding(12)
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }

    private func sectionHeader(_ title: String) -> some View {
        Text(title.uppercased())
            .font(.system(.caption, design: .monospaced).bold())
            .foregroundColor(.secondary)
    }

    private func row(_ label: String, _ value: String) -> some View {
        HStack {
            Text(label + ":")
                .foregroundColor(.secondary)
            Spacer()
            Text(value)
        }
    }

    private func coloredValue(_ v: Float, threshold: Float) -> some View {
        let text = String(format: "%+.3f", v)
        let color: Color = abs(v) > threshold ? .red : (abs(v) > threshold * 0.5 ? .orange : .primary)
        return Text(text).foregroundColor(color)
    }

    private func uprightIndicator(_ z: Float) -> some View {
        let color: Color = z > 0.9 ? .green : (z > 0.5 ? .yellow : .red)
        let symbol = z > 0.9 ? "✓" : (z > 0.5 ? "⚠" : "✗")
        return Text(symbol).foregroundColor(color)
    }

    private func f3(_ v: SIMD3<Float>) -> String {
        String(format: "(%.3f, %.3f, %.3f)", v.x, v.y, v.z)
    }
}

struct PhysicsControlBar: View {
    @ObservedObject var vm: PhysicsDebugViewModel
    var onInspect: () -> Void
    var onClose: () -> Void

    var body: some View {
        VStack(spacing: 0) {
            Divider()
            HStack(spacing: 12) {
                VStack(alignment: .leading, spacing: 0) {
                    Text("STEP \(vm.stepCount)")
                        .font(.system(.caption2, design: .monospaced).bold())
                    Text(String(format: "T=%.3fs", vm.simTime))
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundColor(.secondary)
                    Text(vm.robotKind.modelDefinition.displayName)
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundColor(.blue)
                    Text(vm.policyKind.displayName)
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundColor(.purple)
                }
                .frame(width: 92, alignment: .leading)

                if vm.canRequestRobotKindSwitch {
                    Menu {
                        ForEach(RobotModelDefinitions.selectable, id: \.kind) { definition in
                            Button {
                                vm.switchRobotKind(definition.kind)
                            } label: {
                                if definition.kind == vm.robotKind {
                                    Label(definition.displayName, systemImage: "checkmark")
                                } else {
                                    Text(definition.displayName)
                                }
                            }
                        }
                    } label: {
                        Label(vm.robotKind.modelDefinition.pickerLabel, systemImage: "cube")
                    }
                    .buttonStyle(.bordered)

                    Menu {
                        ForEach(vm.availablePolicies, id: \.self) { policyKind in
                            Button {
                                vm.switchPolicyKind(policyKind)
                            } label: {
                                if policyKind == vm.policyKind {
                                    Label(policyKind.displayName, systemImage: "checkmark")
                                } else {
                                    Text(policyKind.displayName)
                                }
                            }
                        }
                    } label: {
                        Label(vm.policyKind.pickerLabel, systemImage: "gearshape.2")
                    }
                    .buttonStyle(.bordered)
                    .disabled(vm.availablePolicies.count <= 1)
                }

                HStack(spacing: 8) {
                    controlButton(title: "Step 1", icon: "arrow.right") { vm.stepOnce() }
                    controlButton(title: "10", icon: "forward.fill") { vm.stepN(10) }
                    
                    if vm.isRunning {
                        controlButton(title: "Stop", icon: "pause.fill", color: .red) { vm.stopContinuous() }
                    } else {
                        controlButton(title: "Run", icon: "play.fill", color: .blue) { vm.startContinuous() }
                    }
                }

                Spacer()

                HStack(spacing: 8) {
                    Button(action: onInspect) {
                        Image(systemName: "list.bullet.rectangle.portrait")
                            .font(.system(size: 18, weight: .medium))
                    }
                    .buttonStyle(.bordered)
                    
                    Button(action: { vm.reset() }) {
                        Image(systemName: "arrow.counterclockwise")
                            .font(.system(size: 18, weight: .medium))
                    }
                    .buttonStyle(.bordered)
                    .tint(.orange)
                    
                    Button(action: onClose) {
                        Image(systemName: "xmark.circle.fill")
                            .font(.system(size: 24))
                            .symbolRenderingMode(.hierarchical)
                    }
                    .foregroundColor(.secondary)
                }
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 10)
            .background(.ultraThinMaterial)
        }
    }

    private func controlButton(title: String, icon: String, color: Color = .primary) -> some View {
        Button(action: {}) { // Action handled by higher level
            VStack(spacing: 2) {
                Image(systemName: icon)
                Text(title).font(.system(size: 9, weight: .bold))
            }
            .frame(width: 44, height: 44)
            .background(Color.primary.opacity(0.05))
            .cornerRadius(8)
        }
        .simultaneousGesture(TapGesture().onEnded {
            // Need to pass closure correctly, but for simplicity:
            // This is just a helper, I'll inline the actual buttons above.
        })
        .buttonStyle(.plain)
        .foregroundColor(color)
    }
    
    // Inline version for better closure handling in SwiftUI
    private func controlButton(title: String, icon: String, color: Color = .primary, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            VStack(spacing: 2) {
                Image(systemName: icon)
                    .font(.system(size: 16, weight: .semibold))
                Text(title).font(.system(size: 9, weight: .bold))
            }
            .frame(width: 48, height: 44)
            .background(Color.primary.opacity(0.08))
            .cornerRadius(8)
        }
        .buttonStyle(.plain)
        .foregroundColor(color)
    }
}

struct PhysicsDebugView: View {
    @StateObject private var vm: PhysicsDebugViewModel
    @State private var showingInspector = false
    var onClose: () -> Void

    init(sharedLoop: PolicyPhysicsLoop,
         selection: RobotPolicySelection,
         onClose: @escaping () -> Void,
         onRequestSelectionSwitch: ((RobotPolicySelection) -> Void)? = nil) {
        let model = PhysicsDebugViewModel(sharedLoop: sharedLoop,
                                          selection: selection)
        model.externalSelectionSwitcher = onRequestSelectionSwitch
        _vm = StateObject(wrappedValue: model)
        self.onClose = onClose
    }

    var body: some View {
        PhysicsControlBar(vm: vm, onInspect: {
            showingInspector = true
        }, onClose: {
            vm.resumeLoop()
            onClose()
        })
        .sheet(isPresented: $showingInspector) {
            PhysicsInspectorView(vm: vm)
        }
    }
}

#Preview {
    VStack {
        Spacer()
        PhysicsControlBar(vm: PhysicsDebugViewModel(), onInspect: {}, onClose: {})
    }
}
