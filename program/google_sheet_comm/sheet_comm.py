import gspread

# This class will handle the writing and reading to the google spread sheet "database"
# THE SHEET MUST BE CONFIGURED WITH GOOGLGE CLOUD
# See the google drive and google sheets API documentation to configure it

class Sheet_Comm():

    # Upon calling of the class define the name of the sheet and worksheet to be used
    def __init__(self, sheet, worksheet):

        # Service account has the json file from google cloud
        # It contains the credentials to use it
        self.service = gspread.service_account()

        # Configure the settings to open the worksheet with the given names
        self.sheet = self.service.open(sheet)
        self.work_sheet = self.sheet.worksheet(worksheet)

    def get_all_data(self):

        # To get all data from worksheet
        data = self.work_sheet.get_all_records()
        return data

    def get_cell_value(self, row: int, col: int):

        # Variables for given row and column numbers to get
        self.row = row
        self.col = col

        # To get single cell value
        cell = self.work_sheet.cell(self.row, self.col).value
        return cell

    def insert_single_row(self, data: list, row_num: int):

        # Take data to insert and the row number to insert it into
        self.data = data
        self.row_num = row_num

        # To insert a row
        self.work_sheet.insert_row(self.data, self.row_num)

    def update_cell_value(self, row: int, col: int, value):

        # Variable of entered value to write to row and col
        self.row = row
        self.col = col
        self.value = value

        # To update a cell
        self.work_sheet.update_cell(self.row, self.col, self.value)