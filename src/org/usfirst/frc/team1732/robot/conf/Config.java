package org.usfirst.frc.team1732.robot.conf;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Config {
	public static final Object OBJ = new Object();

	public TalonSRX talon(String path) {
		Object cur = getValue(path, OBJ);

		TalonSRX t;
		if (cur instanceof JSONObject) {
			t = new TalonSRX(getValue(path + ".num", 0));
			if (getValue(path + ".reversed", false)) {
				t.setInverted(true);
			}
		} else if (cur instanceof JSONArray) {
			t = new TalonSRX(getValue(path + ".0.num", 0));
			if (getValue(path + ".0.reversed", false)) {
				t.setInverted(true);
			}
			
			TalonSRX tf;
			JSONArray a = getValue(path, new JSONArray());
			for (int i = 1; i < a.size(); i++) {
				tf = new TalonSRX(((Long) ((JSONObject)a.get(i)).get("num")).intValue());
				if (((JSONObject)a.get(i)).get("reversed") != null) {
					tf.setInverted((Boolean) ((JSONObject)a.get(i)).get("reversed"));
				}
				tf.set(ControlMode.Follower, t.getDeviceID());
			}
		} else {
			t = new TalonSRX(0);
		}

		return t;
	}

	public Joystick joystick(String path) {
		Object cur = getValue(path, OBJ);

		Joystick t;
		if (cur instanceof JSONObject) {
			t = new Joystick(getValue(path + ".usb", 0));
			t.setXChannel(getValue(path + ".x", t.getXChannel()));
			t.setYChannel(getValue(path + ".y", t.getYChannel()));
			t.setZChannel(getValue(path + ".z", t.getZChannel()));
		} else {
			t = new Joystick(0);
		}

		return t;
	}

	public Solenoid piston(String path) {
		Object cur = getValue(path, OBJ);

		Solenoid t;
		if (cur instanceof JSONObject) {
			int i = getValue(path + ".can", -1);
			if (i != -1) {
				t = new Solenoid(i, getValue(path + ".num", 0));
			} else {
				t = new Solenoid(getValue(path + ".num", 0));
			}
		} else {
			t = new Solenoid(0);
		}

		return t;
	}

	public JoystickButton button(String path) {
		Object cur = getValue(path, OBJ);

		JoystickButton t;
		if (cur instanceof JSONObject) {
			t = new JoystickButton(new Joystick(getValue(path + ".usb", 0)), getValue(path + ".num", 0));
		} else {
			t = new JoystickButton(new Joystick(0), 0);
		}

		return t;
	}

	@SuppressWarnings("unchecked")
	public <T> T getValue(String path, T defualt) {
		String[] pathParts = path.split("\\.");
		Object cur = o;
		for (String p : pathParts) {
			if (cur instanceof JSONObject) {
				cur = ((JSONObject) cur).get(p);
			} else if (cur instanceof JSONArray) {
				cur = ((JSONArray) cur).get(Integer.parseInt(p));
			} else {
				return defualt;
			}
		}
		
		try {
			if (defualt instanceof Integer) {
				cur = ((Long) cur).intValue();
			}
			if (cur == null) {
				return defualt;
			}
		
			return (T) cur;
		} catch (Exception e) {
			return defualt;
		}
	}
	
	public static Config load() {
		return new Config();
	}
	
	public static final File LOCAL_FILE = new File("/home/lvuser/confFile.json");
	public static final JSONParser p = new JSONParser();
	public static final String NET_ENTRY = "CONFIG/config";
	public static final NetworkTableInstance ni = NetworkTableInstance.getDefault();
	public static final NetworkTableEntry ne = ni.getEntry(NET_ENTRY);

	private JSONObject o;
	private static boolean loaded = true;

	public Config() {
		try {
			String raw = readFile(LOCAL_FILE);
			ne.forceSetString(raw);
			ne.addListener((__) -> {
				System.out.println(ne.getString("{}"));
				saveFile(LOCAL_FILE, ne.getString("{}"));
			}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
			
			o = (JSONObject) p.parse(raw);
		} catch (ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			loaded = false;
		}
		if(!loaded) {
//			hold();
		}

	}

	public static String readFile(File f) {
		StringBuilder s = new StringBuilder();

		try (BufferedReader br = new BufferedReader(new FileReader(f))) {
			String cur = br.readLine();
			while (cur != null) {
				s.append(cur);
				cur = br.readLine();
			}
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			loaded = false;
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			loaded = false;
		}

		return s.toString();
	}

	public static void saveFile(File f, String s) {
		try (BufferedWriter bw = new BufferedWriter(new FileWriter(f))) {
			bw.write(s);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			loaded = false;
		}
	}

	public static void hold() {
//		while (true) {
//			Thread.yield();
//		}
	}
}
