using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;

namespace _3CPO_v2.ViewModel;

public partial class SettingsViewModel : ObservableObject
{
    [RelayCommand]
    async Task BackTap()
    {
        await Shell.Current.GoToAsync("..");
    }
}
