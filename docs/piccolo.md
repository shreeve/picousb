### Piccolo Interactions

#### Performing a dump of all results

<kbd>
<img width="400" alt="picousb" src="https://github.com/user-attachments/assets/db8f9e49-305f-4923-a211-f480b5d98cd9">
</kbd>

#### Example console output (December 7, 2024)

```
==[ PicoUSB Host ]==

Connected Device:
  Total Length:       18
  Device Class:       0
    Subclass:         0
    Protocol:         0
  Packet Size:        8
  Vendor Id:          0x0403
  Product Id:         0xcd18
  Manufacturer:       [#1]
  Product:            [#2]
  Serial:             [#3]

Configuration Descriptor:
  Total Length:       32
  Interfaces:         1
  Config Value:       1
  Config Name:        [#0]
  Attributes:         Self Powered
  Max Power:          0 mA

Interface Descriptor:
  Interface:          0
  Alternate:          0
  Endpoints:          2
  Class:              0xff
  Subclass:           0xff
  Protocol:           0xff
  Name:               [#2]

Endpoint Descriptor:
  Length:             7
  Endpoint number:    EP1
  Endpoint direction: IN
  Attributes:         0x02 (Bulk Transfer Type)
  Max Packet Size:    64
  Interval:           0

Endpoint Descriptor:
  Length:             7
  Endpoint number:    EP2
  Endpoint direction: OUT
  Attributes:         0x02 (Bulk Transfer Type)
  Max Packet Size:    64
  Interval:           0

[String #1]: "Abaxis Inc."
[String #2]: "piccolo xpress"
[String #3]: "AVP09880"
Device 1 is configured
FTDI device is setup

[Piccolo wants to speak!]
1H|\^&|||ABAXIS, INC.^piccolo xpress^2.1.57^0000P09880|||||||P|E 1394-97|20241207130711
2P|1|18090092||||||U||||||^^|Control||||||||||
3O|1|||^^^Comprehensive Metabolic: 9172BB5||20190724112415|||||||||||||||||||F
4C|1|I|^^INST QC: OK    CHEM QC: OK|G
5C|2|I|^^HEM: -18  LIP: 187  ICT: 3|G
6C|3|I|^^|G
7C|4|I|^^|G
0C|5|I|^^|G
1R|1|2951-2^^LN^Sodium SerPl-sCnc|147|mmol/L||||R
2R|2|2823-3^^LN^Potassium SerPl-sCnc|5.9|mmol/L||||R
3R|3|2028-9^^LN^CO2 SerPl-sCnc|21|mmol/L||||R
4R|4|2075-0^^LN^Chloride SerPl-sCnc|113|mmol/L||||R
5R|5|2345-7^^LN^Glucose SerPl-mCnc|268|mg/dL||||R
6R|6|17861-6^^LN^Calcium SerPl-mCnc|11.6|mg/dL||||R
7R|7|3094-0^^LN^BUN SerPl-mCnc|50|mg/dL||||R
0R|8|2160-0^^LN^Creat SerPl-mCnc|4.9|mg/dL||||R
1R|9|6768-6^^LN^ALP SerPl-cCnc|476|U/L||||R
2R|10|1742-6^^LN^ALT SerPl-cCnc|159|U/L||||R
3R|11|1920-8^^LN^AST SerPl-cCnc|251|U/L||||R
4R|12|1975-2^^LN^Bilirub SerPl-mCnc|3.5|mg/dL||||R
5R|13|1751-7^^LN^Alb SerPl-mCnc|4.3|g/dL||||R
6R|14|2885-2^^LN^Prot SerPl-mCnc|7.4|g/dL||||R
7O|2|||^^^* QUALITY CONTROL REPORT *: 9172BB5||20190724112415|||||||||||||||||||F
0C|1|I|^^CHEMISTRY QC:        96|G
1C|2|I|^^ACCEPTABLE MINIMUM:  50|G
2R|1|^^^LEVEL 1: IQC 1|91||90 to 110|||
3R|2|^^^LEVEL 1: IQC 2|107||90 to 110|||
4R|3|^^^LEVEL 1: IQC 3|92||90 to 110|||
5R|4|^^^LEVEL 1: IQC 4|91||90 to 110|||
6R|5|^^^LEVEL 1: IQC 5|104||90 to 110|||
7R|6|^^^LEVEL 1: IQC 6|90||90 to 110|||
0R|7|^^^LEVEL 1: IQC 7|106||90 to 110|||
1R|8|^^^LEVEL 1: IQC 8|93||90 to 110|||
2R|9|^^^LEVEL 2: PRE|101||95 to 105|||
3R|10|^^^LEVEL 2: 340 nm|100||95 to 105|||
4R|11|^^^LEVEL 2: 405 nm|100||95 to 105|||
5R|12|^^^LEVEL 2: 467 nm|100||95 to 105|||
6R|13|^^^LEVEL 2: 500 nm|100||95 to 105|||
7R|14|^^^LEVEL 2: 515 nm|100||95 to 105|||
0R|15|^^^LEVEL 2: 550 nm|100||95 to 105|||
1R|16|^^^LEVEL 2: 600 nm|100||95 to 105|||
2R|17|^^^LEVEL 2: 630 nm|100||95 to 105|||
3L|1|N
[Piccolo is done with that transfer]

[Piccolo wants to speak!]
1H|\^&|||ABAXIS, INC.^piccolo xpress^2.1.57^0000P09880|||||||P|E 1394-97|20241207130718
2P|1|18090092||||||U||||||^^|Control||||||||||
3O|1|||^^^Comprehensive Metabolic: 9172BB5||20190724121446|||||||||||||||||||F
4C|1|I|^567F FFFF^Run Count\^ADFF FFFF^Abort Count|I
5C|2|I|^222F FFFF^Print Count\^BFED D5FF^Flash Count|I
6C|3|I|^0000^DAC Trim Flags\^0000^Offset Error Flags\^0000 0001^System Flags|I
7C|4|I|^0000 0000^Bead Check 1 Flags\^0000 0000^Bead Check 2 Flags|I
0C|5|I|^0000 0000^Empty Cuvette Flags\^0000 0000^Distribution Check Flags|I
1C|6|I|^0000^Quality Control Flags\^0000^Offset SD Error Flags\^0000^Wavelength CV Flags|I
2C|7|I|^4037^Insufficient Sample Error|I
3L|1|N
[Piccolo is done with that transfer]

[Piccolo wants to speak!]
1H|\^&|||ABAXIS, INC.^piccolo xpress^2.1.57^0000P09880|||||||P|E 1394-97|20241207130720
2P|1|18090092||||||U||||||^^|Control||||||||||
3O|1|||^^^Comprehensive Metabolic: 9172BB5||20190724123918|||||||||||||||||||F
4C|1|I|^^INST QC: OK    CHEM QC: OK|G
5C|2|I|^^HEM: -18  LIP: 186  ICT: 3|G
6C|3|I|^^|G
7C|4|I|^^|G
0C|5|I|^^|G
1R|1|2951-2^^LN^Sodium SerPl-sCnc|149|mmol/L||||R
2R|2|2823-3^^LN^Potassium SerPl-sCnc|6.0|mmol/L||||R
3R|3|2028-9^^LN^CO2 SerPl-sCnc|19|mmol/L||||R
4R|4|2075-0^^LN^Chloride SerPl-sCnc|115|mmol/L||||R
5R|5|2345-7^^LN^Glucose SerPl-mCnc|269|mg/dL||||R
6R|6|17861-6^^LN^Calcium SerPl-mCnc|11.7|mg/dL||||R
7R|7|3094-0^^LN^BUN SerPl-mCnc|50|mg/dL||||R
0R|8|2160-0^^LN^Creat SerPl-mCnc|4.8|mg/dL||||R
1R|9|6768-6^^LN^ALP SerPl-cCnc|476|U/L||||R
2R|10|1742-6^^LN^ALT SerPl-cCnc|152|U/L||||R
3R|11|1920-8^^LN^AST SerPl-cCnc|249|U/L||||R
4R|12|1975-2^^LN^Bilirub SerPl-mCnc|3.6|mg/dL||||R
5R|13|1751-7^^LN^Alb SerPl-mCnc|4.3|g/dL||||R
6R|14|2885-2^^LN^Prot SerPl-mCnc|7.5|g/dL||||R
7O|2|||^^^* QUALITY CONTROL REPORT *: 9172BB5||20190724123918|||||||||||||||||||F
0C|1|I|^^CHEMISTRY QC:        99|G
1C|2|I|^^ACCEPTABLE MINIMUM:  50|G
2R|1|^^^LEVEL 1: IQC 1|91||90 to 110|||
3R|2|^^^LEVEL 1: IQC 2|107||90 to 110|||
4R|3|^^^LEVEL 1: IQC 3|92||90 to 110|||
5R|4|^^^LEVEL 1: IQC 4|91||90 to 110|||
6R|5|^^^LEVEL 1: IQC 5|103||90 to 110|||
7R|6|^^^LEVEL 1: IQC 6|90||90 to 110|||
0R|7|^^^LEVEL 1: IQC 7|106||90 to 110|||
1R|8|^^^LEVEL 1: IQC 8|93||90 to 110|||
2R|9|^^^LEVEL 2: PRE|101||95 to 105|||
3R|10|^^^LEVEL 2: 340 nm|100||95 to 105|||
4R|11|^^^LEVEL 2: 405 nm|100||95 to 105|||
5R|12|^^^LEVEL 2: 467 nm|100||95 to 105|||
6R|13|^^^LEVEL 2: 500 nm|100||95 to 105|||
7R|14|^^^LEVEL 2: 515 nm|100||95 to 105|||
0R|15|^^^LEVEL 2: 550 nm|100||95 to 105|||
1R|16|^^^LEVEL 2: 600 nm|100||95 to 105|||
2R|17|^^^LEVEL 2: 630 nm|100||95 to 105|||
3L|1|N
[Piccolo is done with that transfer]

[Piccolo wants to speak!]
1H|\^&|||ABAXIS, INC.^piccolo xpress^2.1.57^0000P09880|||||||P|E 1394-97|20241207130723
2P|1|ERROR 4056||||||U||||||^^|||||||||||
3O|1|||^^^Unknown:||20220601110245|||||||||||||||||||F
4C|1|I|^567F FFFF^Run Count\^ADFF FFFF^Abort Count|I
5C|2|I|^222F FFFF^Print Count\^BFED D5FF^Flash Count|I
6C|3|I|^0000^DAC Trim Flags\^0000^Offset Error Flags\^0000 0000^System Flags|I
7C|4|I|^0000 0000^Bead Check 1 Flags\^0000 0000^Bead Check 2 Flags|I
0C|5|I|^0000 0000^Empty Cuvette Flags\^0000 0000^Distribution Check Flags|I
1C|6|I|^0000^Quality Control Flags\^0000^Offset SD Error Flags\^0000^Wavelength CV Flags|I
2C|7|I|^4056^Disc Date Illegal|I
3L|1|N
[Piccolo is done with that transfer]

[Piccolo wants to speak!]
1H|\^&|||ABAXIS, INC.^piccolo xpress^2.1.57^0000P09880|||||||P|E 1394-97|20241207130724
2P|1|ERROR 4056||||||U||||||^^|||||||||||
3O|1|||^^^Unknown:||20220601110313|||||||||||||||||||F
4C|1|I|^567F FFFF^Run Count\^ADFF FFFF^Abort Count|I
5C|2|I|^222F FFFF^Print Count\^BFED D5FF^Flash Count|I
6C|3|I|^0000^DAC Trim Flags\^0000^Offset Error Flags\^0000 0000^System Flags|I
7C|4|I|^0000 0000^Bead Check 1 Flags\^0000 0000^Bead Check 2 Flags|I
0C|5|I|^0000 0000^Empty Cuvette Flags\^0000 0000^Distribution Check Flags|I
1C|6|I|^0000^Quality Control Flags\^0000^Offset SD Error Flags\^0000^Wavelength CV Flags|I
2C|7|I|^4056^Disc Date Illegal|I
3L|1|N
[Piccolo is done with that transfer]

[Piccolo wants to speak!]
1H|\^&|||ABAXIS, INC.^piccolo xpress^2.1.57^0000P09880|||||||P|E 1394-97|20241207130725
2P|1|ERROR 4056||||||U||||||^^|Patient||||||||||
3O|1|||^^^Unknown:||20220601111258|||||||||||||||||||F
4C|1|I|^567F FFFF^Run Count\^ADFF FFFF^Abort Count|I
5C|2|I|^222F FFFF^Print Count\^BFED D5FF^Flash Count|I
6C|3|I|^0000^DAC Trim Flags\^0000^Offset Error Flags\^0000 0000^System Flags|I
7C|4|I|^0000 0000^Bead Check 1 Flags\^0000 0000^Bead Check 2 Flags|I
0C|5|I|^0000 0000^Empty Cuvette Flags\^0000 0000^Distribution Check Flags|I
1C|6|I|^0000^Quality Control Flags\^0000^Offset SD Error Flags\^0000^Wavelength CV Flags|I
2C|7|I|^4056^Disc Date Illegal|I
3L|1|N
[Piccolo is done with that transfer]
```
