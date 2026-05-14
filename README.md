# VRP Starter Project

## Project Structure

- `vrp.py`
- `data/Demand_List.csv`
- `data/Dist_Matrix.csv`
- `requirements.txt`

## Setup

1. Create and activate a virtual environment:

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   ```

2. Install dependencies:

   ```bash
   python -m pip install --upgrade pip
   pip install -r requirements.txt
   ```

3. Run:

   ```bash
   python vrp.py
   ```

## Notes

- `vrp.py` loads CSVs from the local `data/` folder using relative paths.
- This keeps the project portable across macOS, Linux, and Windows.
