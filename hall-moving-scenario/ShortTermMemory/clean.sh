# clean every folder that needs to

# sterg clasificatorii
rm -r $(pwd)/ShortTermMemory/classifiers/*

# trebuie sa sterg toate fisierele pickle din fisierul de comunicare\
rm -r $(pwd)/ShortTermMemory/conversations/*

# trebuie sa sterg orice fisier din short_time_memory files care nu trebuia sa fie acolo
find -path "$(pwd)/ShortTermMemory/temporary_aligned_faces_dir/*" -type d -not -name stefania -not -name alex -not -name "." -exec rm -r {} \;
find -path "$(pwd)/ShortTermMemory/temporary_faces_dir/*" -type d -not -name stefania -not -name alex -not -name "." -exec rm -r {} \;

# sa sterg orice din to_align_dir
rm -r $(pwd)/ShortTermMemory/to_align_dir/*
