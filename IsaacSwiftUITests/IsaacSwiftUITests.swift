//
//  IsaacSwiftUITests.swift
//  IsaacSwiftUITests
//
//  Created by Tatsuya Ogawa on 2026/04/29.
//

import XCTest

final class IsaacSwiftUITests: XCTestCase {

    override func setUpWithError() throws {
        continueAfterFailure = false
    }

    @MainActor
    func testRendererLaunches() throws {
        let app = XCUIApplication()
        app.launch()

        XCTAssertTrue(app.otherElements["renderer-view"].waitForExistence(timeout: 10))
        XCTAssertFalse(app.staticTexts["renderer-status-label"].exists)
    }
}
