//
//  FCUEfisControlView.swift
//  WinwingDesktop
//
//  Created by Ramon Swilem on 30/07/2025.
//

import SwiftUI

struct FCUEfisControlView: View {
    let device: WinwingDevice
    @State private var backlight: Double = 0
    @State private var screenBacklight: Double = 0
    @State private var efisRightBacklight: Double = 0
    @State private var efisRightScreenBacklight: Double = 0
    @State private var efisLeftBacklight: Double = 0
    @State private var efisLeftScreenBacklight: Double = 0
    
    // LED states for FCU-EFIS indicators
    @State private var fcuLedStates: [Bool] = Array(repeating: false, count: 31)
    @State private var efisRightLedStates: [Bool] = Array(repeating: false, count: 10)
    @State private var efisLeftLedStates: [Bool] = Array(repeating: false, count: 10)
    @State private var selectedTestDisplay: String = "ALL"
    @State private var efisLeftSelectedTestDisplay: String = "QNH_1013"
    @State private var efisRightSelectedTestDisplay: String = "QNH_1013"
    
    private let fcuIndicatorLEDs: [(id: Int, name: String)] = [
        (3, "LOC"), (5, "AP1"), (7, "AP2"), (9, "ATHR"), 
        (11, "EXPED"), (13, "APPR"), (17, "FLAG"), (30, "EXPED YEL")
    ]
    
    private let efisRightIndicatorLEDs: [(id: Int, name: String)] = [
        (102, "FLAG"), (103, "FD"), (104, "LS"), (105, "CSTR"),
        (106, "WPT"), (107, "VOR/D"), (108, "NDB"), (109, "ARPT")
    ]
    
    private let efisLeftIndicatorLEDs: [(id: Int, name: String)] = [
        (202, "FLAG"), (203, "FD"), (204, "LS"), (205, "CSTR"),
        (206, "WPT"), (207, "VOR/D"), (208, "NDB"), (209, "ARPT")
    ]
    
    var body: some View {
        VStack(alignment: .leading, spacing: 20) {
            Text("FCU-EFIS Controls")
                .font(.headline)
                .padding(.top, 8)
            
            // Backlight Controls
            VStack(alignment: .leading, spacing: 12) {
                Text("Backlights")
                    .font(.subheadline)
                    .fontWeight(.medium)
                
                HStack(alignment: .center, spacing: 16) {
                    Text("FCU Backlight")
                        .frame(width: 120, alignment: .leading)
                    Slider(value: $backlight, in: 0...255, step: 1)
                        .frame(width: 140)
                    Text("\(Int(backlight))")
                        .frame(width: 36, alignment: .trailing)
                    Button(action: { setBacklight() }) {
                        Text("Set")
                    }
                    .buttonStyle(.bordered)
                }
                
                HStack(alignment: .center, spacing: 16) {
                    Text("FCU Screen")
                        .frame(width: 120, alignment: .leading)
                    Slider(value: $screenBacklight, in: 0...255, step: 1)
                        .frame(width: 140)
                    Text("\(Int(screenBacklight))")
                        .frame(width: 36, alignment: .trailing)
                    Button(action: { setScreenBacklight() }) {
                        Text("Set")
                    }
                    .buttonStyle(.bordered)
                }
                
                HStack(alignment: .center, spacing: 16) {
                    Text("EFIS Right")
                        .frame(width: 120, alignment: .leading)
                    Slider(value: $efisRightBacklight, in: 0...255, step: 1)
                        .frame(width: 140)
                    Text("\(Int(efisRightBacklight))")
                        .frame(width: 36, alignment: .trailing)
                    Button(action: { setEfisRightBacklight() }) {
                        Text("Set")
                    }
                    .buttonStyle(.bordered)
                }
              
              HStack(alignment: .center, spacing: 16) {
                  Text("EFIS Right Screen")
                      .frame(width: 120, alignment: .leading)
                  Slider(value: $efisRightScreenBacklight, in: 0...255, step: 1)
                      .frame(width: 140)
                  Text("\(Int(efisRightScreenBacklight))")
                      .frame(width: 36, alignment: .trailing)
                  Button(action: { setEfisRightScreenBacklight() }) {
                      Text("Set")
                  }
                  .buttonStyle(.bordered)
              }
                
                HStack(alignment: .center, spacing: 16) {
                    Text("EFIS Left")
                        .frame(width: 120, alignment: .leading)
                    Slider(value: $efisLeftBacklight, in: 0...255, step: 1)
                        .frame(width: 140)
                    Text("\(Int(efisLeftBacklight))")
                        .frame(width: 36, alignment: .trailing)
                    Button(action: { setEfisLeftBacklight() }) {
                        Text("Set")
                    }
                    .buttonStyle(.bordered)
                }
              
              HStack(alignment: .center, spacing: 16) {
                  Text("EFIS Left Screen")
                      .frame(width: 120, alignment: .leading)
                  Slider(value: $efisLeftScreenBacklight, in: 0...255, step: 1)
                      .frame(width: 140)
                  Text("\(Int(efisLeftScreenBacklight))")
                      .frame(width: 36, alignment: .trailing)
                  Button(action: { setEfisLeftScreenBacklight() }) {
                      Text("Set")
                  }
                  .buttonStyle(.bordered)
              }
            }
            
            // Display Test Controls
            VStack(alignment: .leading, spacing: 12) {
                Text("Display Tests")
                    .font(.subheadline)
                    .fontWeight(.medium)
              
                  HStack(spacing: 12) {
                      Picker("EFIS Left Display", selection: $efisLeftSelectedTestDisplay) {
                          Text("QNH 1013").tag("QNH_1013")
                          Text("QNH 29.92").tag("QNH_2992")
                          Text("STD").tag("STD")
                      }
                      .pickerStyle(MenuPickerStyle())
                      
                      Button(action: { efisLeftTestDisplay(efisLeftSelectedTestDisplay) }) {
                          Text("Test Display")
                      }
                      .buttonStyle(.borderedProminent)
                      
                      Button(action: { efisLeftClear() }) {
                          Text("Clear")
                      }
                      .buttonStyle(.bordered)
                      
                      Spacer()
                  }
                }
                
                HStack(spacing: 12) {
                    Picker("FCU Display", selection: $selectedTestDisplay) {
                        Text("All Segments").tag("ALL")
                        Text("Speed Test").tag("SPEED")
                        Text("Heading Test").tag("HEADING")
                        Text("Altitude Test").tag("ALTITUDE")
                        Text("V/S Test").tag("VS")
                        Text("EFIS Right").tag("EFIS_R")
                        Text("EFIS Left").tag("EFIS_L")
                    }
                    .pickerStyle(MenuPickerStyle())
                    
                    Button(action: { fcuTestDisplay(selectedTestDisplay) }) {
                        Text("Test Display")
                    }
                    .buttonStyle(.borderedProminent)
                    
                    Button(action: { clearDisplay() }) {
                        Text("Clear")
                    }
                    .buttonStyle(.bordered)
                    
                    Spacer()
                }
              
                HStack(spacing: 12) {
                    Picker("EFIS Right Display", selection: $efisRightSelectedTestDisplay) {
                        Text("QNH 1013").tag("QNH_1013")
                        Text("QNH 29.92").tag("QNH_2992")
                        Text("STD").tag("STD")
                    }
                    .pickerStyle(MenuPickerStyle())
                    
                    Button(action: { efisRightTestDisplay(efisRightSelectedTestDisplay) }) {
                        Text("Test Display")
                    }
                    .buttonStyle(.borderedProminent)
                    
                    Button(action: { efisRightClear() }) {
                        Text("Clear")
                    }
                    .buttonStyle(.bordered)
                    
                    Spacer()
                }
              
            
            // FCU LED Indicators
            VStack(alignment: .leading, spacing: 12) {
                Text("FCU LED Indicators")
                    .font(.subheadline)
                    .fontWeight(.medium)
                
                LazyVGrid(columns: Array(repeating: GridItem(.flexible()), count: 4), spacing: 8) {
                    ForEach(Array(fcuIndicatorLEDs.enumerated()), id: \.offset) { index, led in
                        Toggle(led.name, isOn: Binding(
                            get: { 
                                if led.id < fcuLedStates.count {
                                    return fcuLedStates[led.id]
                                }
                                return false
                            },
                            set: { newValue in
                                if led.id < fcuLedStates.count {
                                    fcuLedStates[led.id] = newValue
                                    setFCULed(led.id, state: newValue)
                                }
                            }
                        ))
                        .toggleStyle(.switch)
                    }
                }
            }
            
            // EFIS Right LED Indicators
            VStack(alignment: .leading, spacing: 12) {
                Text("EFIS Right LED Indicators")
                    .font(.subheadline)
                    .fontWeight(.medium)
                
                LazyVGrid(columns: Array(repeating: GridItem(.flexible()), count: 4), spacing: 8) {
                    ForEach(Array(efisRightIndicatorLEDs.enumerated()), id: \.offset) { index, led in
                        Toggle(led.name, isOn: Binding(
                            get: { 
                                let arrayIndex = led.id - 102
                                if arrayIndex >= 0 && arrayIndex < efisRightLedStates.count {
                                    return efisRightLedStates[arrayIndex]
                                }
                                return false
                            },
                            set: { newValue in
                                let arrayIndex = led.id - 102
                                if arrayIndex >= 0 && arrayIndex < efisRightLedStates.count {
                                    efisRightLedStates[arrayIndex] = newValue
                                    setEfisRightLed(led.id, state: newValue)
                                }
                            }
                        ))
                        .toggleStyle(.switch)
                    }
                }
            }
            
            // EFIS Left LED Indicators
            VStack(alignment: .leading, spacing: 12) {
                Text("EFIS Left LED Indicators")
                    .font(.subheadline)
                    .fontWeight(.medium)
                
                LazyVGrid(columns: Array(repeating: GridItem(.flexible()), count: 4), spacing: 8) {
                    ForEach(Array(efisLeftIndicatorLEDs.enumerated()), id: \.offset) { index, led in
                        Toggle(led.name, isOn: Binding(
                            get: { 
                                let arrayIndex = led.id - 202
                                if arrayIndex >= 0 && arrayIndex < efisLeftLedStates.count {
                                    return efisLeftLedStates[arrayIndex]
                                }
                                return false
                            },
                            set: { newValue in
                                let arrayIndex = led.id - 202
                                if arrayIndex >= 0 && arrayIndex < efisLeftLedStates.count {
                                    efisLeftLedStates[arrayIndex] = newValue
                                    setEfisLeftLed(led.id, state: newValue)
                                }
                            }
                        ))
                        .toggleStyle(.switch)
                    }
                }
            }
            
            Spacer()
        }
        .onAppear(perform: viewDidAppear)
    }
    
    private func viewDidAppear() {
        setDatarefFloat("AirbusFBW/PanelBrightnessLevel", 1);
        //setDatarefFloat("AirbusFBW/FCUAltitude", "----");
    }
    
    private func setBacklight() {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.setBacklight(UInt8(backlight))
    }
    
    private func setScreenBacklight() {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.setScreenBacklight(UInt8(screenBacklight))
    }
    
    private func setEfisRightBacklight() {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.setEfisRightBacklight(UInt8(efisRightBacklight))
    }
  
  private func setEfisRightScreenBacklight() {
      guard let fcuefis = device.fcuEfis else { return }
      fcuefis.setEfisRightScreenBacklight(UInt8(efisRightScreenBacklight))
  }
    
    private func setEfisLeftBacklight() {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.setEfisLeftBacklight(UInt8(efisLeftBacklight))
    }
  
  private func setEfisLeftScreenBacklight() {
      guard let fcuefis = device.fcuEfis else { return }
      fcuefis.setEfisLeftScreenBacklight(UInt8(efisLeftScreenBacklight))
  }
    
    private func fcuTestDisplay(_ testType: String) {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.testDisplay(testType)
    }
    
    private func efisRightTestDisplay(_ testType: String) {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.efisRightTestDisplay(testType)
    }
    
    private func efisLeftTestDisplay(_ testType: String) {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.efisLeftTestDisplay(testType)
    }
    
    private func efisRightClear() {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.efisRightClear()
    }
    
    private func efisLeftClear() {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.efisLeftClear()
    }
    
    private func clearDisplay() {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.clear()
    }
    
    private func setFCULed(_ ledId: Int, state: Bool) {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.setLed(ledId, state: state)
    }
    
    private func setEfisRightLed(_ ledId: Int, state: Bool) {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.setLed(ledId, state: state)
    }
    
    private func setEfisLeftLed(_ ledId: Int, state: Bool) {
        guard let fcuefis = device.fcuEfis else { return }
        fcuefis.setLed(ledId, state: state)
    }
}

#Preview {
    FCUEfisControlView(device: WinwingDevice(
        id: 0,
        name: "FCU-EFIS Device",
        type: .fcuEfis,
        productId: 0xBA01,
        isConnected: true
    ))
}
