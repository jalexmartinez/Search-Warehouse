######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################

import io
import sys
import unittest
import traceback

try:
    from testing_suite_partA import PartATestCase
    from testing_suite_partB import PartBTestCase
    from testing_suite_partC import PartCTestCase
    from warehouse import who_am_i
    studentExc = None
except Exception as e:
    studentExc = traceback.format_exc()


def run_all( fout ):

    scores = ()
    prcts = ()
    parts = ()

    for part_name, test_case, prct in (
                ('A', PartATestCase, .40),
                ('B', PartBTestCase, .40),
                ('C', PartCTestCase, .20)):
        part_fout = io.StringIO()
        test_case.fout = part_fout

        suite = unittest.TestSuite(unittest.TestLoader().loadTestsFromTestCase(test_case))
        result = unittest.TestResult()
        suite.run(result)

        # No need to truncate here:
        # output already truncated per test case.
        output = part_fout.getvalue()
        score = test_case.totalCredit
        scores += (score,)
        prcts += (prct,)
        parts += (part_name,)

        fout.write( output )
        fout.write( f'part {part_name} result: {score:.02f} [weight = {int(100*prct)}%] | {10*prct*score:.2f}%\n')
        fout.write( f'{"-_"*20}\n    part {part_name} finished    \n{"-_"*20}\n')

    totalScore = sum([s*p for s,p in zip(scores, prcts)]) / 10.0

    fout.write('-----------------------------\n')
    for part, score, prct in zip(parts, scores, prcts):
        fout.write(f'Part {part} result: {score:.02f} [weight = {100 * prct}%] | {10 * prct * score:.2f}%\n')


    if totalScore > 1.01:
        fout.write( "Total score larger than 100%! Truncated to 101%!\n" )
        totalScore = 1.00

    intTotalScore = int( (totalScore * 100) + .5)   # round score up to nearest integer
    if intTotalScore > 100:
        intTotalScore = 100
    fout.write( "score: %d\n" % intTotalScore )


# This flag is used to check whether project files listed in the json have been modified.
# Modifications include (but are not limited to) print statements, changing flag values, etc.
# If you have modified the project files in some way, the results may not be accurate.
# Turn file_checker on by setting the flag to True to ensure you are running against
# the same framework as the Gradescope autograder.
file_checker = False  # set to True to turn file checking on

if file_checker:
    import json
    import hashlib
    import pathlib
    print("File checking is turned on.")
    with open('file_check.json', 'r') as openfile:
        json_dict = json.load(openfile)

    modified_files = []
    for file in json_dict:
        f = str(file)
        try:
            current = pathlib.Path(file).read_text().replace(' ', '').replace('\n', '')
            file_hash = hashlib.sha256(current.encode()).hexdigest()
            if file_hash != json_dict[f]:
                modified_files.append(f)
        except:
            print(f'File ({f}) not in project folder.')

    if len(modified_files) == 0:
        print("You are running against the same framework as the Gradescope autograder.")
    else:
        print("Warning. The following files have been modified and the results may not be accurate:")
        print(", ".join(modified_files))

if __name__ == '__main__':
    if studentExc:
        print(studentExc)
        print('score: 0')
    else:
        student_id = who_am_i()
        if student_id:
            try:
                run_all( sys.stdout )
            except Exception as e:
                print(e)
                print('score: 0')
        else:
            print("Student ID not specified.  Please fill in 'whoami' variable.")
            print('score: 0')
