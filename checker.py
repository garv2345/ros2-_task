import os
import zipfile
import subprocess
import re
import sys
import shutil

# --- CONFIGURATION ---
SAFE_LIMIT_VAL = 3.14
REPORT_FILE = "report.txt"

def check_syntax(file_path):
    """Checks Python syntax using python3 -m flake8 (Safer method)."""
    try:
        cmd = [sys.executable, "-m", "flake8", file_path]
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode == 0:
            return "PASSED"
        else:
            return "FAILED (Syntax Errors)"
    except Exception as e:
        return f"ERROR: {str(e)}"

def check_structure(extracted_folder):
    """Checks if package.xml and CMakeLists.txt/setup.py exist."""
    has_package_xml = False
    has_build_file = False
    
    for root, dirs, files in os.walk(extracted_folder):
        if "package.xml" in files:
            has_package_xml = True
        if "CMakeLists.txt" in files or "setup.py" in files:
            has_build_file = True
            
    if has_package_xml and has_build_file:
        return "PASSED"
    else:
        return "FAILED: Missing package.xml or setup.py/CMakeLists.txt"

def check_safety(file_path):
    """Simple heuristic to check for unsafe values."""
    warnings = []
    with open(file_path, 'r', errors='ignore') as f:
        for i, line in enumerate(f):
            if "while True" in line or "while(true)" in line:
                if "sleep" not in line and "rate" not in line:
                    warnings.append(f"Line {i+1}: Infinite loop detected without sleep.")
            
            matches = re.findall(r'= (\d+\.\d+)', line)
            for match in matches:
                if float(match) > SAFE_LIMIT_VAL:
                    warnings.append(f"Line {i+1}: High value detected ({match}).")

    if not warnings:
        return "PASSED"
    else:
        return "WARNINGS: " + "; ".join(warnings)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 checker.py <path_to_zip_file>")
        return

    zip_path = sys.argv[1]
    extract_to = "temp_extracted"
    
    # Cleanup previous run
    if os.path.exists(extract_to):
        shutil.rmtree(extract_to)

    try:
        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
            zip_ref.extractall(extract_to)
    except Exception as e:
        print(f"Critical Error: Could not unzip file. {e}")
        return

    print("--- ROS CODE REPORT ---")
    print(f"Checking package: {zip_path}\n")
    
    # Scan files
    for root, dirs, files in os.walk(extract_to):
        for file in files:
            if file.endswith(".py"): 
                full_path = os.path.join(root, file)
                print(f"Checking: {file}")
                print(f"Syntax: {check_syntax(full_path)}")
                print(f"Safety: {check_safety(full_path)}")
                print("-" * 20)

    print(f"Structure Check: {check_structure(extract_to)}")
    
    # Cleanup
    if os.path.exists(extract_to):
        shutil.rmtree(extract_to)

if __name__ == "__main__":
    main()
